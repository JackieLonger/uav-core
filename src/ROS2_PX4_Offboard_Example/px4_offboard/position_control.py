#!/usr/bin/env python
############################################################################
#
#   position_control.py - 位置控制版本（基於 velocity_control.py）
#
#   主要差異：
#   - 使用 PoseStamped 接收目標位置（而非 Twist 接收速度）
#   - OffboardControlMode: position=True, velocity=False
#   - TrajectorySetpoint 使用 position[] 欄位
#   - 邊界檢查直接裁剪目標座標
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
############################################################################

__author__ = "Braden Wagstaff (modified for position control)"
__contact__ = "braden@arkelectron.com"

import rclpy
from rclpy.node import Node
import numpy as np
import time
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


class PositionControl(Node):
    """位置控制模式的 Offboard 控制器"""

    def __init__(self):
        super().__init__('position_control')
        
        # PX4 使用的 QoS
        px4_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Optimizer 使用的 QoS
        cmd_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ========== 訂閱 ==========
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            px4_qos_profile)
        
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.position_callback,
            px4_qos_profile)
        
        # ✅ 位置控制：訂閱 PoseStamped（而非 Twist）
        self.offboard_position_sub = self.create_subscription(
            PoseStamped,
            'offboard_position_cmd',  # 相對名稱，允許 remapping
            self.offboard_position_callback,
            cmd_qos_profile)
        
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
        
        self.command_sub = self.create_subscription(
            String,
            'command',
            self.command_callback,
            cmd_qos_profile)

        # ========== 發布 ==========
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos_profile)
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 10)

        # ========== 計時器 ==========
        self.arm_timer_ = self.create_timer(0.1, self.arm_timer_callback)
        self.timer = self.create_timer(0.02, self.cmdloop_callback)  # 50Hz

        # ========== 狀態變數 ==========
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED
        self.trueYaw = 0.0
        self.offboardMode = False
        self.flightCheck = False
        self.myCnt = 0
        self.arm_message = False
        self.failsafe = False
        self.current_state = "IDLE"
        self.last_state = self.current_state
        
        # ✅ 位置控制：目標位置（NED 座標）
        self.target_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.target_yaw = float('nan')  # NaN = 保持當前航向
        self.position_valid = False  # 是否有有效的目標位置
        
        # 超時保護
        self.last_cmd_time = 0.0
        self.cmd_timeout = 5.0
        
        # 狀態機相關
        self.prearm_setpoint_count = 0
        self.PREARM_SETPOINT_REQUIRED = 20
        self.takeoff_target_altitude = 2.5
        
        # 當前位置
        self.current_altitude = 0.0
        self.ground_level_z = None
        self.current_position = None
        
        # 掃描控制
        self.scan_control_pub = self.create_publisher(Bool, 'scan_control', 10)
        
        # RC 接管檢測
        self.offboard_lost_time = 0.0
        self.RC_TAKEOVER_THRESHOLD = 0.3
        self.rc_offboard_entry = False
        self.takeoff_altitude_reached = False
        
        # ✅ 安全範圍保護（位置模式：直接裁剪目標）- 與 optimizer 統一
        self.offboard_entry_position = None
        self.safety_bounds_x = (-1.6, 1.6)  # ✅ 與速度版本統一 ±1.6m
        self.safety_bounds_y = (-1.6, 1.6)  # ✅ 與速度版本統一 ±1.6m
        self.safety_bounds_z = (0.0, 3.5)   # ✅ 與速度版本統一 3.5m
        self.boundary_check_enabled = True
        
        # ✅ 空中重啟檢測標記
        self.airborne_restart_detected = False
        
        # RViz2 可視化
        self.origin_pub = self.create_publisher(PoseStamped, 'safety_origin', 10)
        self.position_pub = self.create_publisher(PoseStamped, 'current_position', 10)
        self.target_pub = self.create_publisher(PoseStamped, 'target_position', 10)
        self.boundary_pub = self.create_publisher(MarkerArray, 'safety_boundary', 10)
        self.path_pub = self.create_publisher(Path, 'vehicle_path', 10)
        
        self.position_history = deque(maxlen=50)
        self.vehicle_path_msg = Path()
        
        self.get_logger().info("🎯 位置控制節點已啟動 (Position Control Mode)")

    # ========== Callbacks ==========
    
    def arm_message_callback(self, msg):
        self.arm_message = msg.data
        self.get_logger().info(f"Arm Message: {self.arm_message}")
    
    def command_callback(self, msg: String):
        """接收鍵盤控制命令"""
        command = msg.data
        self.get_logger().info(f"收到命令: {command}")
        
        if command == 'ARM_TOGGLE':
            if self.arm_state == 2:
                self.get_logger().info("無人機已解鎖，若要降落請按 L")
            else:
                self.arm_message = True
                self.get_logger().info("🚀 觸發自動起飛流程：ARM → TAKEOFF(2.5m) → OFFBOARD")
        
        elif command == 'LAND':
            self.arm_message = False
            self.current_state = "IDLE"
            self.offboardMode = False
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.get_logger().info("🛬 發送降落命令")
        
        elif command == 'HOLD':
            # 緊急懸停：設定目標為當前位置
            if self.current_position is not None:
                self.target_position = {
                    'x': self.current_position['x'],
                    'y': self.current_position['y'],
                    'z': self.current_position['z']
                }
                self.position_valid = True
            self.get_logger().info("⏸️ 緊急懸停：鎖定當前位置")
        
        elif command == 'START_SCAN':
            scan_cmd = Bool()
            scan_cmd.data = True
            self.scan_control_pub.publish(scan_cmd)
            self.get_logger().info("📡 發送啟動掃描命令")
        
        elif command == 'STOP_SCAN':
            scan_cmd = Bool()
            scan_cmd.data = False
            self.scan_control_pub.publish(scan_cmd)
            self.get_logger().info("⏹️  發送停止掃描命令")
    
    def position_callback(self, msg: VehicleLocalPosition):
        """接收位置信息"""
        # ✅ 只在未解鎖時設定地面參考點
        if (self.ground_level_z is None and 
            msg.z != 0.0 and
            self.arm_state != VehicleStatus.ARMING_STATE_ARMED):
            self.ground_level_z = msg.z
            self.get_logger().info(f"地面參考點設定: Z = {self.ground_level_z:.2f}m (NED)")
        
        # ✅ 若已解鎖但 ground_level_z 仍為空，判定為空中重啟
        elif (self.ground_level_z is None and 
              msg.z != 0.0 and
              self.arm_state == VehicleStatus.ARMING_STATE_ARMED):
            self.ground_level_z = msg.z
            self.airborne_restart_detected = True
            self.get_logger().warning(
                f"⚠️ 檢測到空中重啟！當前高度 {-msg.z:.2f}m，禁止自動起飛"
            )
            # 強制跳過起飛流程，進入 OFFBOARD
            if self.current_state in ["IDLE", "PREARM", "ARMING", "TAKEOFF"]:
                self.current_state = "OFFBOARD"
                self.arm_message = False
        
        if self.ground_level_z is not None:
            self.current_altitude = self.ground_level_z - msg.z
        
        self.current_position = {'x': msg.x, 'y': msg.y, 'z': msg.z}
    
    def offboard_position_callback(self, msg: PoseStamped):
        """✅ 接收目標位置（PoseStamped，NED 座標）"""
        self.target_position = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z
        }
        
        # 從四元數提取 yaw（如果有效）
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        
        # 檢查是否為有效的四元數（不是全零）
        if qw != 0.0 or qx != 0.0 or qy != 0.0 or qz != 0.0:
            # 計算 yaw（NED 座標系）
            self.target_yaw = np.arctan2(2.0 * (qw * qz + qx * qy),
                                          1.0 - 2.0 * (qy * qy + qz * qz))
        else:
            self.target_yaw = float('nan')  # 保持當前航向
        
        self.position_valid = True
        self.last_cmd_time = time.time()
    
    def attitude_callback(self, msg):
        """接收姿態信息"""
        orientation_q = msg.q
        self.trueYaw = -(np.arctan2(
            2.0 * (orientation_q[3] * orientation_q[0] + orientation_q[1] * orientation_q[2]),
            1.0 - 2.0 * (orientation_q[0] * orientation_q[0] + orientation_q[1] * orientation_q[1])
        ))
    
    def vehicle_status_callback(self, msg):
        """接收飛控狀態"""
        if msg.nav_state != self.nav_state:
            self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")
        if msg.arming_state != self.arm_state:
            self.get_logger().info(f"ARM STATUS: {msg.arming_state}")
        if msg.failsafe != self.failsafe:
            self.get_logger().info(f"FAILSAFE: {msg.failsafe}")
        if msg.pre_flight_checks_pass != self.flightCheck:
            self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")

        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass
        
        # RC 切入 Offboard 檢測
        if msg.nav_state == 14 and not self.offboardMode:
            self.get_logger().info("✅ 偵測到 RC 切入 Offboard 模式")
            self.offboardMode = True
            self.current_state = "OFFBOARD"
            self.rc_offboard_entry = True
            self.offboard_lost_time = 0.0
            
            if self.current_position is not None:
                self.offboard_entry_position = dict(self.current_position)
                # 初始化目標為當前位置
                self.target_position = dict(self.current_position)
                self.position_valid = True
                self.get_logger().info(
                    f"🎯 安全範圍原點已設定: "
                    f"X={self.offboard_entry_position['x']:.2f}m, "
                    f"Y={self.offboard_entry_position['y']:.2f}m, "
                    f"Z={self.offboard_entry_position['z']:.2f}m"
                )
        
        elif self.offboardMode and msg.nav_state != 14:
            current_time = time.time()
            if msg.nav_state in [3, 17]:
                self.offboard_lost_time = 0.0
            else:
                if self.offboard_lost_time == 0.0:
                    self.offboard_lost_time = current_time
                elif (current_time - self.offboard_lost_time) >= self.RC_TAKEOVER_THRESHOLD:
                    self.get_logger().info(f"📻 RC 接管 (nav_state={msg.nav_state})")
                    self.offboardMode = False
                    self.rc_offboard_entry = False
        
        elif self.offboardMode and msg.nav_state == 14:
            self.offboard_lost_time = 0.0

    # ========== 狀態機 ==========
    
    def arm_timer_callback(self):
        """狀態機主循環"""
        if self.myCnt % 50 == 0:
            self.get_logger().info(
                f"[狀態機] 當前: {self.current_state} | "
                f"arm_msg: {self.arm_message} | "
                f"arm_state: {self.arm_state} | "
                f"flightCheck: {self.flightCheck}"
            )

        match self.current_state:
            case "IDLE":
                if self.flightCheck and self.arm_message:
                    self.current_state = "PREARM"
                    self.prearm_setpoint_count = 0
                    self.get_logger().info("🔄 IDLE → PREARM")
            
            case "PREARM":
                if not self.arm_message:
                    self.current_state = "IDLE"
                elif self.prearm_setpoint_count >= self.PREARM_SETPOINT_REQUIRED:
                    if self.flightCheck:
                        self.current_state = "ARMING"
                        self.get_logger().info("✅ PREARM → ARMING")
                    else:
                        self.prearm_setpoint_count = 0
                else:
                    self.prearm_setpoint_count += 1

            case "ARMING":
                if not self.arm_message:
                    self.current_state = "IDLE"
                elif not self.flightCheck:
                    self.current_state = "IDLE"
                elif self.arm_state == 2 and self.myCnt > 10:
                    self.current_state = "TAKEOFF"
                    self.get_logger().info("✅ ARMING → TAKEOFF")
                self.arm()

            case "TAKEOFF":
                if not self.arm_message:
                    self.current_state = "IDLE"
                elif not self.flightCheck:
                    self.current_state = "IDLE"
                elif self.arm_state != 2:
                    self.current_state = "IDLE"
                elif self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
                    self.current_state = "LOITER"
                    self.get_logger().info("✅ TAKEOFF → LOITER")
                elif self.myCnt > 100:
                    self.current_state = "LOITER"
                self.arm()
                self.take_off()

            case "LOITER":
                if not self.arm_message:
                    self.current_state = "IDLE"
                elif not self.flightCheck:
                    self.current_state = "IDLE"
                elif self.arm_state != 2:
                    self.current_state = "IDLE"
                elif self.current_altitude >= (self.takeoff_target_altitude - 0.5):
                    self.current_state = "OFFBOARD"
                    self.get_logger().info(f"✅ LOITER → OFFBOARD（高度: {self.current_altitude:.2f}m）")
                    if self.current_position is not None:
                        self.offboard_entry_position = dict(self.current_position)
                        self.target_position = dict(self.current_position)
                        self.position_valid = True
                        self.get_logger().info(
                            f"🎯 安全範圍原點已設定: "
                            f"X={self.offboard_entry_position['x']:.2f}m, "
                            f"Y={self.offboard_entry_position['y']:.2f}m, "
                            f"高度={self.current_altitude:.2f}m"
                        )
                elif self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                    self.current_state = "OFFBOARD"
                elif self.myCnt > 500:
                    self.current_state = "OFFBOARD"
                else:
                    if self.myCnt % 50 == 0:
                        self.get_logger().info(f"🚁 起飛中... {self.current_altitude:.2f}m / {self.takeoff_target_altitude}m")
                self.arm()

            case "OFFBOARD":
                if self.arm_state != 2:
                    self.current_state = "IDLE"
                    self.arm_message = False
                    self.offboardMode = False
                elif self.failsafe:
                    self.current_state = "IDLE"
                    self.arm_message = False
                    self.offboardMode = False
                elif not self.offboardMode:
                    self.state_offboard()
                    self.get_logger().info("🎮 Offboard 模式已啟用（位置控制）")
                else:
                    self.state_offboard()

        if self.current_state == "IDLE" and self.arm_state != 2:
            self.arm_message = False

        if self.last_state != self.current_state:
            self.last_state = self.current_state

        self.myCnt += 1

    # ========== 控制命令 ==========
    
    def state_offboard(self):
        self.myCnt = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        self.offboardMode = True

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def take_off(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=1.0, param7=2.5)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)

    # ========== 主控制迴圈（位置模式）==========
    
    def cmdloop_callback(self):
        """發布位置控制指令"""
        
        # TAKEOFF/LOITER 期間不發送任何控制指令
        if self.current_state in ["TAKEOFF", "LOITER"]:
            self.publish_safety_visualization()
            return
        
        # 決定目標位置
        if self.current_state == "PREARM":
            # PREARM：發送當前位置（懸停）
            if self.current_position is not None:
                tx, ty, tz = self.current_position['x'], self.current_position['y'], self.current_position['z']
            else:
                tx, ty, tz = 0.0, 0.0, 0.0
            tyaw = float('nan')
            
        elif self.offboardMode and self.current_state == "OFFBOARD":
            # OFFBOARD：使用接收到的目標位置
            current_time = time.time()
            
            if not self.position_valid:
                # 尚未收到目標，懸停在當前位置
                if self.current_position is not None:
                    tx, ty, tz = self.current_position['x'], self.current_position['y'], self.current_position['z']
                else:
                    tx, ty, tz = 0.0, 0.0, 0.0
                tyaw = float('nan')
            elif self.last_cmd_time > 0 and (current_time - self.last_cmd_time) > self.cmd_timeout:
                # 超時：懸停在當前位置
                self.get_logger().warning("位置指令超時，懸停在當前位置")
                if self.current_position is not None:
                    tx, ty, tz = self.current_position['x'], self.current_position['y'], self.current_position['z']
                else:
                    tx, ty, tz = 0.0, 0.0, 0.0
                tyaw = float('nan')
            else:
                tx = self.target_position['x']
                ty = self.target_position['y']
                tz = self.target_position['z']
                tyaw = self.target_yaw
        else:
            # 其他狀態：懸停
            if self.current_position is not None:
                tx, ty, tz = self.current_position['x'], self.current_position['y'], self.current_position['z']
            else:
                tx, ty, tz = 0.0, 0.0, 0.0
            tyaw = float('nan')
        
        # ✅ 邊界檢查（直接裁剪目標座標）
        if (self.offboardMode and 
            self.current_state == "OFFBOARD" and
            self.offboard_entry_position is not None and 
            self.boundary_check_enabled):
            tx, ty, tz = self.clamp_target_position(tx, ty, tz)
        
        # ✅ 發送 OffboardControlMode（位置模式）
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True   # ← 位置控制
        offboard_msg.velocity = False  # ← 關閉速度控制
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        self.publisher_offboard_mode.publish(offboard_msg)

        # ✅ 發送 TrajectorySetpoint（位置模式）
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        
        # 位置（NED）
        trajectory_msg.position[0] = tx
        trajectory_msg.position[1] = ty
        trajectory_msg.position[2] = tz
        
        # 速度設為 NaN（不控制）
        trajectory_msg.velocity[0] = float('nan')
        trajectory_msg.velocity[1] = float('nan')
        trajectory_msg.velocity[2] = float('nan')
        
        # 加速度設為 NaN
        trajectory_msg.acceleration[0] = float('nan')
        trajectory_msg.acceleration[1] = float('nan')
        trajectory_msg.acceleration[2] = float('nan')
        
        # Yaw
        trajectory_msg.yaw = tyaw
        trajectory_msg.yawspeed = float('nan')

        self.publisher_trajectory.publish(trajectory_msg)
        
        self.publish_safety_visualization()
    
    def clamp_target_position(self, tx: float, ty: float, tz: float) -> tuple:
        """
        ✅ 位置模式邊界檢查：直接裁剪目標座標
        
        返回：裁剪後的 (tx, ty, tz)
        """
        origin_x = self.offboard_entry_position['x']
        origin_y = self.offboard_entry_position['y']
        origin_z = self.offboard_entry_position['z']
        
        # X 軸裁剪
        min_x = origin_x + self.safety_bounds_x[0]
        max_x = origin_x + self.safety_bounds_x[1]
        if tx < min_x:
            tx = min_x
            self.get_logger().warning(f"⚠️ X 軸目標裁剪至下限 {min_x:.2f}m")
        elif tx > max_x:
            tx = max_x
            self.get_logger().warning(f"⚠️ X 軸目標裁剪至上限 {max_x:.2f}m")
        
        # Y 軸裁剪
        min_y = origin_y + self.safety_bounds_y[0]
        max_y = origin_y + self.safety_bounds_y[1]
        if ty < min_y:
            ty = min_y
            self.get_logger().warning(f"⚠️ Y 軸目標裁剪至下限 {min_y:.2f}m")
        elif ty > max_y:
            ty = max_y
            self.get_logger().warning(f"⚠️ Y 軸目標裁剪至上限 {max_y:.2f}m")
        
        # Z 軸裁剪（NED：向上 = z 減小）
        max_altitude = self.safety_bounds_z[1]  # 最大上升高度
        min_z = origin_z - max_altitude  # NED 中 z 越小 = 越高
        max_z = origin_z  # 不允許低於原點
        
        if tz < min_z:
            tz = min_z
            self.get_logger().warning(f"⚠️ Z 軸目標裁剪至上限 {max_altitude:.1f}m")
        elif tz > max_z:
            tz = max_z
            self.get_logger().warning(f"⚠️ Z 軸目標裁剪至下限（禁止下降）")
        
        return (tx, ty, tz)
    
    def publish_safety_visualization(self):
        """✅ 發布可視化數據（NED → ENU 轉換）"""
        if self.offboard_entry_position is None or self.current_position is None:
            return
        
        now = self.get_clock().now().to_msg()
        frame_id = 'map'
        
        # ✅ NED → ENU 轉換函數
        def ned_to_enu(ned_x, ned_y, ned_z):
            """NED (North, East, Down) → ENU (East, North, Up)"""
            enu_x = ned_y      # East = NED East
            enu_y = ned_x      # North = NED North
            enu_z = -ned_z     # Up = -NED Down
            return enu_x, enu_y, enu_z
        
        # 原點
        origin_pose = PoseStamped()
        origin_pose.header.stamp = now
        origin_pose.header.frame_id = frame_id
        o_x, o_y, o_z = ned_to_enu(
            self.offboard_entry_position['x'],
            self.offboard_entry_position['y'],
            self.offboard_entry_position['z']
        )
        origin_pose.pose.position.x = o_x
        origin_pose.pose.position.y = o_y
        origin_pose.pose.position.z = o_z
        origin_pose.pose.orientation.w = 1.0
        self.origin_pub.publish(origin_pose)
        
        # 當前位置
        current_pose = PoseStamped()
        current_pose.header.stamp = now
        current_pose.header.frame_id = frame_id
        c_x, c_y, c_z = ned_to_enu(
            self.current_position['x'],
            self.current_position['y'],
            self.current_position['z']
        )
        current_pose.pose.position.x = c_x
        current_pose.pose.position.y = c_y
        current_pose.pose.position.z = c_z
        current_pose.pose.orientation.w = 1.0
        self.position_pub.publish(current_pose)
        
        # ✅ 目標位置
        if self.position_valid:
            target_pose = PoseStamped()
            target_pose.header.stamp = now
            target_pose.header.frame_id = frame_id
            t_x, t_y, t_z = ned_to_enu(
                self.target_position['x'],
                self.target_position['y'],
                self.target_position['z']
            )
            target_pose.pose.position.x = t_x
            target_pose.pose.position.y = t_y
            target_pose.pose.position.z = t_z
            target_pose.pose.orientation.w = 1.0
            self.target_pub.publish(target_pose)
        
        # 軌跡
        self.vehicle_path_msg.header = current_pose.header
        self.vehicle_path_msg.poses.append(current_pose)
        self.path_pub.publish(self.vehicle_path_msg)
        
        self.position_history.append(dict(self.current_position))
        
        # 邊界框 (ENU 座標)
        marker_array = MarkerArray()
        boundary_marker = Marker()
        boundary_marker.header.stamp = now
        boundary_marker.header.frame_id = frame_id
        boundary_marker.id = 0
        boundary_marker.type = Marker.CUBE
        boundary_marker.action = Marker.ADD
        b_x, b_y, b_z = ned_to_enu(
            self.offboard_entry_position['x'],
            self.offboard_entry_position['y'],
            self.offboard_entry_position['z'] - 1.5
        )
        boundary_marker.pose.position.x = b_x
        boundary_marker.pose.position.y = b_y
        boundary_marker.pose.position.z = b_z
        boundary_marker.pose.orientation.w = 1.0
        boundary_marker.scale.x = 3.2  # ✅ 對應 ±1.6m (2 × 1.6m)
        boundary_marker.scale.y = 3.2  # ✅ 對應 ±1.6m
        boundary_marker.scale.z = 3.5  # ✅ 對應 0~3.5m 高度
        boundary_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.3)
        boundary_marker.lifetime = Duration(seconds=1).to_msg()
        marker_array.markers.append(boundary_marker)
        
        # 軌跡線條
        if len(self.position_history) > 1:
            trajectory_marker = Marker()
            trajectory_marker.header.stamp = now
            trajectory_marker.header.frame_id = frame_id
            trajectory_marker.id = 1
            trajectory_marker.type = Marker.LINE_STRIP
            trajectory_marker.action = Marker.ADD
            
            for pos in self.position_history:
                p_x, p_y, p_z = ned_to_enu(pos['x'], pos['y'], pos['z'])
                point = Point()
                point.x = p_x
                point.y = p_y
                point.z = p_z
                trajectory_marker.points.append(point)
            
            trajectory_marker.scale.x = 0.01
            trajectory_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.7)
            trajectory_marker.lifetime = Duration(seconds=5).to_msg()
            marker_array.markers.append(trajectory_marker)
        
        self.boundary_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = PositionControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
