#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Braden Wagstaff"
__contact__ = "braden@arkelectron.com"

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from geometry_msgs.msg import Twist, Vector3
from math import pi
from std_msgs.msg import Bool


class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        # PX4 使用的 QoS（用於 PX4 topics）
        px4_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Optimizer 使用的 QoS（用於接收速度指令）
        cmd_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        #Create subscriptions
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            px4_qos_profile)
        
        # 訂閱本地位置（用於判斷高度）
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.position_callback,
            px4_qos_profile)
        
        # ✅ 使用相對 topic 名稱 + 匹配的 QoS
        self.offboard_velocity_sub = self.create_subscription(
            Twist,
            'offboard_velocity_cmd',  # 相對名稱，允許 remapping 到 /drone_X/offboard_velocity_cmd
            self.offboard_velocity_callback,
            cmd_qos_profile)  # 使用與 optimizer 匹配的 QoS
        
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            px4_qos_profile)
        
        self.my_bool_sub = self.create_subscription(
            Bool,
            '/arm_message',
            self.arm_message_callback,
            px4_qos_profile)


        #Create publishers
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos_profile)
        self.publisher_velocity = self.create_publisher(Twist, '/fmu/in/setpoint_velocity/cmd_vel_unstamped', px4_qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        
        #creates callback function for the arm timer
        # period is arbitrary, just should be more than 2Hz
        arm_timer_period = .1 # seconds
        self.arm_timer_ = self.create_timer(arm_timer_period, self.arm_timer_callback)

        # creates callback function for the command loop
        # period is arbitrary, just should be more than 2Hz. Because live controls rely on this, a higher frequency is recommended
        # commands in cmdloop_callback won't be executed if the vehicle is not in offboard mode
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED
        self.velocity = Vector3()
        self.yaw = 0.0  #yaw value we send as command
        self.trueYaw = 0.0  #current yaw value of drone
        self.offboardMode = False
        self.flightCheck = False
        self.myCnt = 0
        self.arm_message = False
        self.failsafe = False
        self.current_state = "IDLE"
        self.last_state = self.current_state
        
        # 超時保護（防止 optimizer 崩潰後無人機繼續移動）
        self.last_cmd_time = 0.0
        self.cmd_timeout = 5.0  # 5 秒超時
        
        # RC 切入 Offboard 標記（用於判斷是否需要自動起飛）
        self.rc_offboard_entry = False
        self.takeoff_altitude_reached = False
        
        # 當前高度（相對於地面，單位：米）
        self.current_altitude = 0.0
        self.ground_level_z = None  # 地面 Z 座標（NED 框架）


    def arm_message_callback(self, msg):
        self.arm_message = msg.data
        self.get_logger().info(f"Arm Message: {self.arm_message}")
    
    def position_callback(self, msg: VehicleLocalPosition):
        """接收位置信息，計算當前高度"""
        # 初始化地面參考點（第一次接收位置時）
        if self.ground_level_z is None and msg.z != 0.0:
            self.ground_level_z = msg.z
            self.get_logger().info(f"地面參考點設定: Z = {self.ground_level_z:.2f}m (NED)")
        
        # 計算相對於地面的高度（NED: Z 向下為正，所以高度 = ground_z - current_z）
        if self.ground_level_z is not None:
            self.current_altitude = self.ground_level_z - msg.z

    #callback function that arms, takes off, and switches to offboard mode
    #implements a finite state machine
    def arm_timer_callback(self):

        match self.current_state:
            case "IDLE":
                if(self.flightCheck and self.arm_message == True):
                    self.current_state = "ARMING"
                    self.get_logger().info(f"Arming")

            case "ARMING":
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Arming, Flight Check Failed")
                elif(self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.myCnt > 10):
                    self.current_state = "TAKEOFF"
                    self.get_logger().info(f"Arming, Takeoff")
                self.arm() #send arm command

            case "TAKEOFF":
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Takeoff, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF):
                    self.current_state = "LOITER"
                    self.get_logger().info(f"Takeoff, Loiter")
                self.arm() #send arm command
                self.take_off() #send takeoff command

            # waits in this state while taking off, and the 
            # moment VehicleStatus switches to Loiter state it will switch to offboard
            case "LOITER": 
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Loiter, Flight Check Failed")
                elif(self.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LOITER and self.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF):
                    # User interrupted takeoff/loiter sequence (e.g. switched to Position)
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Loiter, User Interrupted (nav_state={self.nav_state}) -> IDLE")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):
                    self.current_state = "OFFBOARD"
                    self.get_logger().info(f"Loiter, Offboard")
                elif(self.myCnt > 50):
                    self.get_logger().warning("LOITER超时，强制进入OFFBOARD")
                    self.current_state = "OFFBOARD"
                self.arm()

            case "OFFBOARD":
                if(not(self.flightCheck) or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe == True):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Offboard, Flight Check Failed")
                self.state_offboard()

        if(self.arm_state != VehicleStatus.ARMING_STATE_ARMED):
            self.arm_message = False

        if (self.last_state != self.current_state):
            self.last_state = self.current_state
            self.get_logger().info(self.current_state)

        self.myCnt += 1

    def state_offboard(self):
        self.myCnt = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        self.offboardMode = True   

    # Arms the vehicle
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")

    # Takes off the vehicle to a user specified altitude (meters)
    def take_off(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7=2.5) # param7 is altitude in meters (修改為 2.5m)
        self.get_logger().info("Takeoff command send (Target Altitude: 2.5m)")

    #publishes command to /fmu/in/vehicle_command
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7    # altitude value in takeoff command
        msg.command = command  # command ID
        msg.target_system = 1  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)

    #receives and sets vehicle status values 
    def vehicle_status_callback(self, msg):

        if (msg.nav_state != self.nav_state):
            self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")
        
        if (msg.arming_state != self.arm_state):
            self.get_logger().info(f"ARM STATUS: {msg.arming_state}")

        if (msg.failsafe != self.failsafe):
            self.get_logger().info(f"FAILSAFE: {msg.failsafe}")
        
        if (msg.pre_flight_checks_pass != self.flightCheck):
            self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")

        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass
        
        # ✅ 偵測遙控器切入 Offboard 模式
        # NAVIGATION_STATE_OFFBOARD = 14
        if msg.nav_state == 14 and not self.offboardMode:
            # 遙控器切入 Offboard，自動啟用控制
            self.get_logger().info("✅ 偵測到 RC 切入 Offboard 模式")
            self.offboardMode = True
            self.current_state = "OFFBOARD"  # 跳過狀態機
            self.rc_offboard_entry = True  # 標記為 RC 切入
            
            # 判斷是否需要起飛：如果當前高度 < 0.5m，認為在地面
            if self.arm_state == VehicleStatus.ARMING_STATE_ARMED:
                if self.current_altitude < 0.5:
                    self.get_logger().info(f"🚁 檢測到地面起飛 (高度: {self.current_altitude:.2f}m)，執行自動起飛至 2.5m")
                    self.take_off()
                    self.takeoff_altitude_reached = False
                else:
                    self.get_logger().info(f"✈️ 檢測到空中切入 (高度: {self.current_altitude:.2f}m)，當前位置將作為安全原點")
                    self.takeoff_altitude_reached = True  # 已經在空中，不需要起飛
            else:
                self.get_logger().warning("⚠️ 無人機未解鎖，請先解鎖後再切入 Offboard")
                
        elif self.offboardMode and msg.nav_state != 14:
            # RC 接管，停止 Offboard
            self.get_logger().warning(
                f"偵測到模式切換 (nav_state={msg.nav_state})，RC 已接管，停止 Offboard"
            )
            self.offboardMode = False
            self.current_state = "IDLE"
            self.rc_offboard_entry = False
            self.takeoff_altitude_reached = False
            self.velocity.x = 0.0
            self.velocity.y = 0.0
            self.velocity.z = 0.0


    #receives Twist commands - 直接使用 NED 座標（optimizer 發送的是 NED）
    def offboard_velocity_callback(self, msg):
        import time
        # ✅ 直接使用 NED 座標（不做 FLU 轉換）
        # Optimizer 發送的已經是 NED 座標：
        #   X = North, Y = East, Z = Down (正值=下降)
        self.velocity.x = msg.linear.x  # NED North
        self.velocity.y = msg.linear.y  # NED East  
        self.velocity.z = msg.linear.z  # NED Down
        self.yaw = msg.angular.z
        
        # 記錄時間戳（用於超時保護）
        self.last_cmd_time = time.time()

    #receives current trajectory values from drone and grabs the yaw value of the orientation
    def attitude_callback(self, msg):
        orientation_q = msg.q

        #trueYaw is the drones current yaw value
        self.trueYaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]), 
                                  1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])))
        
    #publishes offboard control modes and velocity as trajectory setpoints
    def cmdloop_callback(self):
        import time
        
        # ✅ 關鍵修正：無條件發送 Setpoints（滿足 PX4 Offboard 切換前提條件）
        # PX4 規定：切換到 Offboard 前必須已收到 > 2Hz 的控制指令
        # 因此我們始終發送，不管當前是否在 Offboard 模式
        
        # 決定發送的速度值
        if self.offboardMode:
            # Offboard 模式：發送真實速度指令
            # 超時檢查（5 秒沒收到指令 → 自動懸停）
            current_time = time.time()
            if self.last_cmd_time > 0 and (current_time - self.last_cmd_time) > self.cmd_timeout:
                self.get_logger().warning(
                    f"速度指令超時 ({current_time - self.last_cmd_time:.1f}s)，自動懸停"
                )
                vx = vy = vz = vyaw = 0.0
            else:
                vx = self.velocity.x
                vy = self.velocity.y
                vz = self.velocity.z
                vyaw = self.yaw
        else:
            # 非 Offboard 模式：發送懸停指令 (0,0,0)
            # 這確保 PX4 隨時準備好接受切換，不會因為沒收到指令而拒絕
            vx = vy = vz = vyaw = 0.0
        
        # ✅ 始終發送 OffboardControlMode（心跳包）
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = False
        offboard_msg.velocity = True
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)            

        # ✅ 始終發送 TrajectorySetpoint
        # 直接使用 NED 座標（optimizer 已經發送 NED）
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        trajectory_msg.velocity[0] = vx  # NED North
        trajectory_msg.velocity[1] = vy  # NED East
        trajectory_msg.velocity[2] = vz  # NED Down
        trajectory_msg.position[0] = float('nan')
        trajectory_msg.position[1] = float('nan')
        trajectory_msg.position[2] = float('nan')
        trajectory_msg.acceleration[0] = float('nan')
        trajectory_msg.acceleration[1] = float('nan')
        trajectory_msg.acceleration[2] = float('nan')
        trajectory_msg.yaw = float('nan')
        trajectory_msg.yawspeed = vyaw

        self.publisher_trajectory.publish(trajectory_msg)


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
