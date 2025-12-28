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
from geometry_msgs.msg import Twist, Vector3, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool, String, ColorRGBA
from nav_msgs.msg import Path
from math import pi
from collections import deque
from rclpy.duration import Duration


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
        
        # 订阅命令（键盘控制）
        self.command_sub = self.create_subscription(
            String,
            'command',  # 相对名称，允许 remapping 到 /drone_X/command
            self.command_callback,
            cmd_qos_profile)


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
        
        # ✅ 新增：預發送 setpoint 計數器（確保 PX4 準備好接收 Offboard）
        self.prearm_setpoint_count = 0
        self.PREARM_SETPOINT_REQUIRED = 20  # 需要發送 20 次 (約 2 秒 @10Hz)
        
        # ✅ 新增：自動起飛目標高度
        self.takeoff_target_altitude = 2.5  # 起飛到 2.5 米懸停
        
        # 當前高度（相對於地面，單位：米）
        self.current_altitude = 0.0
        self.ground_level_z = None  # 地面 Z 座標（NED 框架）
        
        # 扫描控制發布器（用於控制 fast_scan_node）
        self.scan_control_pub = self.create_publisher(
            Bool,
            'scan_control',  # 相對名稱，映射到 /drone_N/scan_control
            10
        )
        
        # ✅ RC 接管檢測：用時間戳來過濾短暫波動
        self.offboard_lost_time = 0.0  # 偵測到失去 Offboard 的時間
        self.RC_TAKEOVER_THRESHOLD = 0.3  # 需要持續 0.3 秒才認為是 RC 接管
        
        # ✅ 安全範圍保護
        self.offboard_entry_position = None  # 進入 Offboard 時的 3D 位置
        self.current_position = None  # 當前無人機位置
        self.safety_bounds_x = (-1.5, 1.5)  # X 軸範圍（相對原點，米）
        self.safety_bounds_y = (-1.5, 1.5)  # Y 軸範圍（相對原點，米）
        self.safety_bounds_z = (0.0, 3.0)  # Z 軸範圍（相對高度，米）
        self.boundary_check_enabled = True  # 邊界檢查開關
        
        # RViz2 可視化發布器
        self.origin_pub = self.create_publisher(
            PoseStamped,
            'safety_origin',
            10
        )
        self.position_pub = self.create_publisher(
            PoseStamped,
            'current_position',
            10
        )
        self.boundary_pub = self.create_publisher(
            MarkerArray,
            'safety_boundary',
            10
        )
        
        self.path_pub = self.create_publisher(
            Path,
            'vehicle_path',
            10
        )
        
        # 位置歷史記錄（用於軌跡顯示）
        self.position_history = deque(maxlen=50)
        self.vehicle_path_msg = Path()


    def arm_message_callback(self, msg):
        self.arm_message = msg.data
        self.get_logger().info(f"Arm Message: {self.arm_message}")
    
    def command_callback(self, msg: String):
        """接收键盘控制命令 - 触发状态机而非直接发送命令"""
        command = msg.data
        self.get_logger().info(f"收到命令: {command}")
        
        if command == 'ARM_TOGGLE':
            # SPACE 键：触发自动流程 ARM → TAKEOFF → OFFBOARD
            if self.arm_state == 2:  # VehicleStatus.ARMING_STATE_ARMED = 2
                # 已在空中，忽略（用户应按 L 降落）
                self.get_logger().info("无人机已解锁，若要降落请按 L")
            else:
                # 触发状态机：IDLE → ARMING → TAKEOFF → LOITER → OFFBOARD
                self.arm_message = True
                self.get_logger().info("🚀 触发自动起飞流程：ARM → TAKEOFF(2.5m) → OFFBOARD")
        
        elif command == 'LAND':
            # L 键：降落并重置状态机
            self.arm_message = False  # 重置状态机
            self.current_state = "IDLE"
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.get_logger().info("🛬 发送降落命令，状态机重置")
        
        elif command == 'HOLD':
            # H 键：紧急悬停（立即清零速度）
            self.velocity.x = 0.0
            self.velocity.y = 0.0
            self.velocity.z = 0.0
            self.yaw = 0.0
            self.get_logger().info("⏸️ 紧急悬停：速度清零")
        
        elif command == 'START_SCAN':
            # F 键：启动扫描
            scan_cmd = Bool()
            scan_cmd.data = True
            self.scan_control_pub.publish(scan_cmd)
            self.get_logger().info("📡 发送启动扫描命令")
        
        elif command == 'STOP_SCAN':
            # G 键：停止扫描
            scan_cmd = Bool()
            scan_cmd.data = False
            self.scan_control_pub.publish(scan_cmd)
            self.get_logger().info("⏹️  发送停止扫描命令")
    
    def position_callback(self, msg: VehicleLocalPosition):
        """接收位置信息，計算當前高度"""
        # 初始化地面參考點（第一次接收位置時）
        if self.ground_level_z is None and msg.z != 0.0:
            self.ground_level_z = msg.z
            self.get_logger().info(f"地面參考點設定: Z = {self.ground_level_z:.2f}m (NED)")
        
        # 計算相對於地面的高度（NED: Z 向下為正，所以高度 = ground_z - current_z）
        if self.ground_level_z is not None:
            self.current_altitude = self.ground_level_z - msg.z
        
        # ✅ 記錄完整的 3D 位置（NED）
        self.current_position = {
            'x': msg.x,
            'y': msg.y,
            'z': msg.z
        }

    #callback function that arms, takes off, and switches to offboard mode
    #implements a finite state machine
    def arm_timer_callback(self):
        # ✅ 每 1 秒打印一次狀態（方便調試）
        if self.myCnt % 50 == 0:
            self.get_logger().info(
                f"[狀態機] 當前: {self.current_state} | "
                f"arm_msg: {self.arm_message} | "
                f"arm_state: {self.arm_state} | "
                f"flightCheck: {self.flightCheck} | "
                f"nav_state: {self.nav_state}"
            )

        match self.current_state:
            case "IDLE":
                if(self.flightCheck and self.arm_message == True):
                    # ✅ 進入 PREARM 狀態，開始預發送 setpoint
                    self.current_state = "PREARM"
                    self.prearm_setpoint_count = 0
                    self.get_logger().info("🔄 IDLE → PREARM（預發送 Offboard Setpoints）")
            
            # ✅ 新增 PREARM 狀態：持續發送 setpoint 直到 PX4 準備好
            case "PREARM":
                if not self.arm_message:
                    # 用戶取消
                    self.current_state = "IDLE"
                    self.prearm_setpoint_count = 0
                    self.get_logger().info("❌ PREARM → IDLE（用戶取消）")
                elif self.prearm_setpoint_count >= self.PREARM_SETPOINT_REQUIRED:
                    # 已發送足夠的 setpoint，檢查 flightCheck
                    if self.flightCheck:
                        self.current_state = "ARMING"
                        self.get_logger().info("✅ PREARM → ARMING（pre_flight_checks_pass=True）")
                    else:
                        # flightCheck 仍為 False，繼續等待
                        self.get_logger().warning(f"⏳ PREARM 等待 flightCheck（當前: {self.flightCheck}，已發送 {self.prearm_setpoint_count} 次）")
                        self.prearm_setpoint_count = 0  # 重置計數器繼續發送
                else:
                    self.prearm_setpoint_count += 1
                    if self.prearm_setpoint_count % 10 == 0:
                        self.get_logger().info(f"PREARM: 已發送 {self.prearm_setpoint_count}/{self.PREARM_SETPOINT_REQUIRED} 次 setpoint")

            case "ARMING":
                if not self.arm_message:
                    self.current_state = "IDLE"
                    self.get_logger().info("❌ ARMING → IDLE（用戶取消）")
                elif(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info("❌ ARMING → IDLE（flightCheck失敗）")
                elif(self.arm_state == 2 and self.myCnt > 10):  # ARMING_STATE_ARMED = 2
                    self.current_state = "TAKEOFF"
                    self.get_logger().info("✅ ARMING → TAKEOFF（已解鎖）")
                self.arm() #send arm command

            case "TAKEOFF":
                if not self.arm_message:
                    self.current_state = "IDLE"
                    self.get_logger().info("❌ TAKEOFF → IDLE（用戶取消）")
                elif(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info("❌ TAKEOFF → IDLE（flightCheck失敗）")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF):
                    self.current_state = "LOITER"
                    self.get_logger().info("✅ TAKEOFF → LOITER（PX4進入起飛模式）")
                self.arm() #send arm command
                self.take_off() #send takeoff command

            # waits in this state while taking off, and the 
            # moment VehicleStatus switches to Loiter state or reaches target altitude, it will switch to offboard
            case "LOITER": 
                if not self.arm_message:
                    self.current_state = "IDLE"
                    self.get_logger().info("❌ LOITER → IDLE（用戶取消）")
                elif(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info("❌ LOITER → IDLE（flightCheck失敗）")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):
                    # 到達 Loiter 狀態，切換到 Offboard
                    self.current_state = "OFFBOARD"
                    self.get_logger().info(f"✅ LOITER → OFFBOARD（PX4進入LOITER，高度: {self.current_altitude:.2f}m）")
                elif self.current_altitude >= (self.takeoff_target_altitude - 0.3):
                    # 已達目標高度（容錯 0.3m），直接切換到 Offboard
                    self.current_state = "OFFBOARD"
                    self.get_logger().info(f"✅ LOITER → OFFBOARD（已達目標高度 {self.current_altitude:.2f}m）")
                elif(self.myCnt > 300):  # 30 秒超時（給足時間起飛）
                    self.get_logger().warning(f"⚠️ LOITER 超時（高度: {self.current_altitude:.2f}m, nav_state: {self.nav_state}），強制 → OFFBOARD")
                    self.current_state = "OFFBOARD"
                self.arm()

            case "OFFBOARD":
                # ✅ 只在嚴重錯誤時退出 Offboard（允許 RC 隨時接管）
                if self.arm_state != 2:  # ARMING_STATE_ARMED = 2
                    self.current_state = "IDLE"
                    self.arm_message = False
                    self.get_logger().info("Offboard: 無人機已 Disarm，返回 IDLE")
                elif self.failsafe:
                    self.current_state = "IDLE"
                    self.arm_message = False
                    self.get_logger().warning("Offboard: Failsafe 觸發，返回 IDLE")
                else:
                    # 持續發送 Offboard 模式指令
                    self.state_offboard()

        # ✅ 修正：只在 IDLE 狀態且無人機未解鎖時重置 arm_message
        # 這樣 PREARM、ARMING、TAKEOFF 等狀態不會被意外取消
        if self.current_state == "IDLE" and self.arm_state != 2:
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
            self.offboard_lost_time = 0.0  # 重置 RC 接管時間計數
            
            # ✅ 記錄安全範圍原點
            if self.current_position is not None:
                self.offboard_entry_position = {
                    'x': self.current_position['x'],
                    'y': self.current_position['y'],
                    'z': self.current_position['z']
                }
                self.get_logger().info(
                    f"🎯 安全範圍原點已設定: "
                    f"X={self.offboard_entry_position['x']:.2f}m, "
                    f"Y={self.offboard_entry_position['y']:.2f}m, "
                    f"Z={self.offboard_entry_position['z']:.2f}m (NED)"
                )
                self.get_logger().info(
                    f"📏 允許移動範圍: "
                    f"X∈[{self.safety_bounds_x[0]}, {self.safety_bounds_x[1]}]m, "
                    f"Y∈[{self.safety_bounds_y[0]}, {self.safety_bounds_y[1]}]m, "
                    f"Z: 向上 0~3m"
                )
            else:
                self.get_logger().warning("⚠️ 無法設定原點：當前位置數據未就緒")
            
            # 判斷是否需要起飛：如果當前高度 < 0.5m，認為在地面
            if self.arm_state == VehicleStatus.ARMING_STATE_ARMED:
                if self.current_altitude < 0.5:
                    self.get_logger().info(f"🚁 檢測到地面起飛 (高度: {self.current_altitude:.2f}m)，執行自動起飛至 {self.takeoff_target_altitude}m")
                    self.take_off()
                    self.takeoff_altitude_reached = False
                else:
                    self.get_logger().info(f"✈️ 檢測到空中切入 (高度: {self.current_altitude:.2f}m)，當前位置將作為安全原點")
                    self.takeoff_altitude_reached = True  # 已經在空中，不需要起飛
            else:
                self.get_logger().warning("⚠️ 無人機未解鎖，請先解鎖後再切入 Offboard")
                
        elif self.offboardMode and msg.nav_state != 14:
            # ✅ 修正：檢測到失去 Offboard 模式，但排除 TAKEOFF/LOITER 狀態
            # nav_state = 17: AUTO_TAKEOFF
            # nav_state = 3: AUTO_LOITER
            import time
            current_time = time.time()
            
            # 如果是 AUTO_TAKEOFF 或 AUTO_LOITER，不觸發 RC 接管
            if msg.nav_state in [3, 17]:  # LOITER or TAKEOFF
                self.offboard_lost_time = 0.0  # 重置計時器
                # 不需要記錄，這是正常的起飛流程
            else:
                # 首次偵測到失去 Offboard（排除 TAKEOFF/LOITER）
                if self.offboard_lost_time == 0.0:
                    self.offboard_lost_time = current_time
                    self.get_logger().warning(
                        f"⚠️ 偵測到模式變化 (nav_state={msg.nav_state})，等待確認..."
                    )
                # 確認持續時間超過閾值
                elif (current_time - self.offboard_lost_time) >= self.RC_TAKEOVER_THRESHOLD:
                    self.get_logger().info(
                        f"📻 RC 接管 (nav_state={msg.nav_state})，Offboard 暫停，遙控器優先"
                    )
                    self.offboardMode = False
                    self.rc_offboard_entry = False
                    # 清零速度，安全懸停
                    self.velocity.x = 0.0
                    self.velocity.y = 0.0
                    self.velocity.z = 0.0
        
        elif self.offboardMode and msg.nav_state == 14:
            # ✅ 重新進入 Offboard，重置計時器
            self.offboard_lost_time = 0.0


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
        
        # ✅ 決定發送的速度值（根據狀態機狀態）
        if self.current_state == "PREARM":
            # PREARM 狀態：發送懸停指令 (0,0,0)，讓 PX4 準備好
            vx = vy = vz = vyaw = 0.0
            
        elif self.offboardMode:
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
        
        # ✅ 邊界檢查（在 Offboard 模式下且原點已設定）
        if (self.offboardMode and 
            self.offboard_entry_position is not None and 
            self.current_position is not None and 
            self.boundary_check_enabled):
            vx, vy, vz = self.check_safety_boundary(vx, vy, vz)
        
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
        
        # ✅ 發布安全範圍可視化數據給 RViz2
        self.publish_safety_visualization()
    
    def check_safety_boundary(self, vx: float, vy: float, vz: float) -> tuple:
        """
        邊界安全檢查：預測下一時刻位置，超出範圍則清零速度
        
        邊界定義（相對於 offboard_entry_position）：
        - X 軸：±1.5m
        - Y 軸：±1.5m
        - Z 軸：0~3m（NED 座標系：向上飛 = Z 減小）
        
        返回：修正後的 (vx, vy, vz)
        """
        dt = 0.1  # 預測時間（秒）
        
        # 當前位置（NED）
        cur_x = self.current_position['x']
        cur_y = self.current_position['y']
        cur_z = self.current_position['z']
        
        # 原點位置（NED）
        origin_x = self.offboard_entry_position['x']
        origin_y = self.offboard_entry_position['y']
        origin_z = self.offboard_entry_position['z']
        
        # 預測下一時刻的絕對位置
        next_x = cur_x + vx * dt
        next_y = cur_y + vy * dt
        next_z = cur_z + vz * dt
        
        # 計算相對於原點的偏移
        offset_x = next_x - origin_x
        offset_y = next_y - origin_y
        # NED: Z 向下為正，所以相對高度 = origin_z - current_z（正值表示在原點上方）
        relative_altitude = origin_z - next_z
        
        # X 軸邊界檢查
        if offset_x < self.safety_bounds_x[0]:
            vx = 0.0
            self.get_logger().warning(
                f"⚠️ X 軸越界（下限）：偏移 {offset_x:.2f}m < {self.safety_bounds_x[0]}m，速度清零"
            )
        elif offset_x > self.safety_bounds_x[1]:
            vx = 0.0
            self.get_logger().warning(
                f"⚠️ X 軸越界（上限）：偏移 {offset_x:.2f}m > {self.safety_bounds_x[1]}m，速度清零"
            )
        
        # Y 軸邊界檢查
        if offset_y < self.safety_bounds_y[0]:
            vy = 0.0
            self.get_logger().warning(
                f"⚠️ Y 軸越界（下限）：偏移 {offset_y:.2f}m < {self.safety_bounds_y[0]}m，速度清零"
            )
        elif offset_y > self.safety_bounds_y[1]:
            vy = 0.0
            self.get_logger().warning(
                f"⚠️ Y 軸越界（上限）：偏移 {offset_y:.2f}m > {self.safety_bounds_y[1]}m，速度清零"
            )
        
        # Z 軸邊界檢查（只允許向上 0~3m，不允許下降到原點以下）
        if vz > 0:  # NED: vz > 0 代表想要下降
            if relative_altitude <= self.safety_bounds_z[0]:
                vz = 0.0
                self.get_logger().warning(
                    f"⚠️ Z 軸越界（下限）：相對高度 {relative_altitude:.2f}m ≤ {self.safety_bounds_z[0]}m，禁止下降"
                )
        elif vz < 0:  # NED: vz < 0 代表想要上升
            if relative_altitude >= self.safety_bounds_z[1]:
                vz = 0.0
                self.get_logger().warning(
                    f"⚠️ Z 軸越界（上限）：相對高度 {relative_altitude:.2f}m ≥ {self.safety_bounds_z[1]}m，禁止上升"
                )
        
        return (vx, vy, vz)
    
    def publish_safety_visualization(self):
        """發布安全範圍數據給 RViz2 可視化"""
        
        # 檢查前提條件
        if self.offboard_entry_position is None or self.current_position is None:
            return
        
        now = self.get_clock().now().to_msg()
        frame_id = 'map'  # 或 'odom'，須與 RViz2 配置一致
        
        # ========== 1. 發布原點位置（綠色球體）==========
        origin_pose = PoseStamped()
        origin_pose.header.stamp = now
        origin_pose.header.frame_id = frame_id
        
        origin_pose.pose.position.x = self.offboard_entry_position['x']
        origin_pose.pose.position.y = self.offboard_entry_position['y']
        origin_pose.pose.position.z = self.offboard_entry_position['z']
        
        # 設置方向為單位四元數（無旋轉）
        origin_pose.pose.orientation.x = 0.0
        origin_pose.pose.orientation.y = 0.0
        origin_pose.pose.orientation.z = 0.0
        origin_pose.pose.orientation.w = 1.0
        
        self.origin_pub.publish(origin_pose)
        
        # ========== 2. 發布當前位置（紅色球體）==========
        current_pose = PoseStamped()
        current_pose.header.stamp = now
        current_pose.header.frame_id = frame_id
        
        current_pose.pose.position.x = self.current_position['x']
        current_pose.pose.position.y = self.current_position['y']
        current_pose.pose.position.z = self.current_position['z']
        
        current_pose.pose.orientation.x = 0.0
        current_pose.pose.orientation.y = 0.0
        current_pose.pose.orientation.z = 0.0
        current_pose.pose.orientation.w = 1.0
        
        self.position_pub.publish(current_pose)
        
        # ========== 3. 發布軌跡（參考 visualizer.py）==========
        # 參考 visualizer.py 的方式發布 Path 訊息
        self.vehicle_path_msg.header = current_pose.header
        self.vehicle_path_msg.poses.append(current_pose)
        self.path_pub.publish(self.vehicle_path_msg)
        
        # 更新位置歷史（用於軌跡）
        self.position_history.append({
            'x': self.current_position['x'],
            'y': self.current_position['y'],
            'z': self.current_position['z']
        })
        
        # ========== 4. 發布安全邊界框（透明藍色立方體）==========
        marker_array = MarkerArray()
        
        boundary_marker = Marker()
        boundary_marker.header.stamp = now
        boundary_marker.header.frame_id = frame_id
        
        boundary_marker.id = 0
        boundary_marker.type = Marker.CUBE
        boundary_marker.action = Marker.ADD
        
        # 位置：邊界中心（原點 + 向下 1.5m，NED 座標系）
        boundary_marker.pose.position.x = self.offboard_entry_position['x']
        boundary_marker.pose.position.y = self.offboard_entry_position['y']
        boundary_marker.pose.position.z = self.offboard_entry_position['z'] - 1.5  # 中心點
        
        boundary_marker.pose.orientation.x = 0.0
        boundary_marker.pose.orientation.y = 0.0
        boundary_marker.pose.orientation.z = 0.0
        boundary_marker.pose.orientation.w = 1.0
        
        # 尺寸：3×3×3 米
        boundary_marker.scale.x = 3.0
        boundary_marker.scale.y = 3.0
        boundary_marker.scale.z = 3.0
        
        # 顏色：藍色，透明度 30%
        boundary_marker.color = ColorRGBA()
        boundary_marker.color.r = 0.0
        boundary_marker.color.g = 0.0
        boundary_marker.color.b = 1.0
        boundary_marker.color.a = 0.3
        
        # 生命週期：1 秒後自動消失（防止重複堆積）
        boundary_marker.lifetime = Duration(seconds=1).to_msg()
        
        marker_array.markers.append(boundary_marker)
        
        # ========== 4. 發布軌跡線條（可選）==========
        if len(self.position_history) > 1:
            trajectory_marker = Marker()
            trajectory_marker.header.stamp = now
            trajectory_marker.header.frame_id = frame_id
            
            trajectory_marker.id = 1
            trajectory_marker.type = Marker.LINE_STRIP
            trajectory_marker.action = Marker.ADD
            
            # 填充軌跡點
            for pos in self.position_history:
                point = Point()
                point.x = pos['x']
                point.y = pos['y']
                point.z = pos['z']
                trajectory_marker.points.append(point)
            
            # 線條樣式：白色，線寬 1cm
            trajectory_marker.scale.x = 0.01
            trajectory_marker.color = ColorRGBA()
            trajectory_marker.color.r = 1.0
            trajectory_marker.color.g = 1.0
            trajectory_marker.color.b = 1.0
            trajectory_marker.color.a = 0.7
            
            trajectory_marker.lifetime = Duration(seconds=5).to_msg()
            
            marker_array.markers.append(trajectory_marker)
        
        self.boundary_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
