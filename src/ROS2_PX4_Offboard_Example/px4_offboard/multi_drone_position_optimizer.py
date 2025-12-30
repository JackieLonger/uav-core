#!/usr/bin/env python3
"""
Multi-Drone Position Optimizer - 多无人机协同信号优化器（位置控制版本）

架构：
- 笔记本运行此节点（集中决策）
- 每个 Jetson 运行 fast_scan_node.py（扫描并发布到 /drone_N/link_quality）
- 每个 Jetson 运行 position_control.py（接收 /drone_N/offboard_position_cmd）
- 使用 PoseStamped 发送目标位置（而非 Twist 发送速度）

主要差異（與 multi_drone_signal_optimizer.py 相比）：
- 發布 PoseStamped 到 /drone_N/offboard_position_cmd（而非 Twist 到 offboard_velocity_cmd）
- 移動邏輯改為「直接設定目標位置」（而非「計算速度」）
- 降低發布頻率（10Hz 而非 100Hz）
- 到達判定由 position_control.py 內部處理，此處只需監控距離

策略：
1. 五點掃描：原點、前、後、左、右
2. RSSI + SNR 綜合評分（各占50%）
3. 選擇最佳點並移動
4. 邊界限制 ±1.5m

键盘控制：（與速度控制版本相同）
- SPACE: ARM/DISARM 所有无人机
- L: 降落
- H: 紧急悬停
- F: 开始扫描
- G: 停止扫描
- S: 开始优化
- P: 暂停优化
- 1/2/3: 选择单个无人机
- A: 选择所有无人机
- Q: 退出程序
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from px4_msgs.msg import VehicleLocalPosition
import threading
import json
import time
import math
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Dict, Tuple, Set

# 键盘控制（Linux/Unix）
if sys.platform != 'win32':
    import termios
    import tty
    import select


@dataclass
class TrackerData:
    """单个 Tracker 的数据"""
    target_id: str
    forward_rssi: float
    forward_snr: float
    return_rssi: float
    return_snr: float
    timestamp: str
    status: str
    
    @property
    def quality_score(self) -> float:
        """综合质量评分（RSSI 50% + SNR 50%）"""
        try:
            forward_rssi = float(self.forward_rssi) if self.forward_rssi else 0.0
            return_rssi = float(self.return_rssi) if self.return_rssi else 0.0
            forward_snr = float(self.forward_snr) if self.forward_snr else 0.0
            return_snr = float(self.return_snr) if self.return_snr else 0.0
            
            rssi_avg = (forward_rssi + return_rssi) / 2.0
            rssi_score = (rssi_avg + 120) / 90.0
            rssi_score = max(0.0, min(1.0, rssi_score))
            
            snr_avg = (forward_snr + return_snr) / 2.0
            snr_score = (snr_avg + 20) / 35.0
            snr_score = max(0.0, min(1.0, snr_score))
            
            return 0.5 * rssi_score + 0.5 * snr_score
        except (ValueError, TypeError):
            return 0.0


@dataclass
class Position:
    """3D位置"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    
    def distance_to(self, other: 'Position') -> float:
        return math.sqrt(
            (self.x - other.x)**2 + 
            (self.y - other.y)**2 + 
            (self.z - other.z)**2
        )


# 掃描序列狀態枚舉
class ScanPhase:
    INIT = "INIT"
    ORIGIN_SCAN = "ORIGIN_SCAN"
    MOVE_FRONT = "MOVE_FRONT"
    FRONT_SCAN = "FRONT_SCAN"
    MOVE_BACK = "MOVE_BACK"
    BACK_SCAN = "BACK_SCAN"
    MOVE_LEFT = "MOVE_LEFT"
    LEFT_SCAN = "LEFT_SCAN"
    MOVE_RIGHT = "MOVE_RIGHT"
    RIGHT_SCAN = "RIGHT_SCAN"
    CHOOSE_BEST = "CHOOSE_BEST"
    MOVE_TO_BEST = "MOVE_TO_BEST"
    HOVERING = "HOVERING"
    DONE = "DONE"


@dataclass
class DroneState:
    """单个无人机的状态（线程安全）"""
    drone_id: int
    
    # Tracker 数据
    dynamic_tracker_ids: list = field(default_factory=list)
    tracker_data_map: Dict[str, TrackerData] = field(default_factory=dict)
    tracker_ready_map: Dict[str, bool] = field(default_factory=dict)
    
    # 位置信息
    current_position: Position = field(default_factory=Position)
    takeoff_position: Optional[Position] = None
    
    # 决策状态
    previous_quality: float = 0.0
    current_quality: float = 0.0
    movement_count: int = 0
    
    # 线程锁
    lock: threading.Lock = field(default_factory=threading.Lock)
    
    # ✅ 位置控制：目標位置（而非速度）
    target_position: Optional[Tuple[float, float, float]] = None
    
    # 质量历史
    quality_history: deque = field(default_factory=lambda: deque(maxlen=10))
    
    # 掃描序列狀態
    scan_phase: str = ScanPhase.INIT
    origin_xy: Optional[Tuple[float, float]] = None
    origin_z: float = 0.0  # ✅ 新增：記錄原點 Z 座標
    scan_scores: Dict[str, float] = field(default_factory=dict)
    best_direction: Optional[str] = None
    is_moving: bool = False
    scan_start_time: float = 0.0
    move_start_time: float = 0.0
    last_distance: float = float('inf')      # ✅ 上次距離（穩定判定）
    distance_stable_count: int = 0           # ✅ 距離穩定計數
    
    def both_trackers_ready(self) -> bool:
        with self.lock:
            if len(self.dynamic_tracker_ids) < 2:
                return False
            return all(self.tracker_ready_map.get(tid, False) for tid in self.dynamic_tracker_ids)
    
    def reset_tracker_flags(self):
        with self.lock:
            for tid in self.dynamic_tracker_ids:
                self.tracker_ready_map[tid] = False
    
    def update_tracker_data(self, json_data: dict):
        with self.lock:
            status = json_data.get('status', 'Unknown')
            if status not in ['Success']:
                return

            target_id = json_data['target_id']
            
            if target_id not in self.dynamic_tracker_ids and len(self.dynamic_tracker_ids) < 2:
                self.dynamic_tracker_ids.append(target_id)
            
            if target_id not in self.dynamic_tracker_ids:
                return
            
            def safe_float(val, default=0.0):
                if val is None:
                    return default
                if isinstance(val, (int, float)):
                    return float(val)
                if isinstance(val, str):
                    val_str = val.strip().upper()
                    if val_str in ('N/A', 'NA', 'NONE', ''):
                        return default
                    try:
                        return float(val)
                    except ValueError:
                        return default
                return default
            
            tracker_data = TrackerData(
                target_id=target_id,
                forward_rssi=safe_float(json_data.get('forward_rssi')),
                forward_snr=safe_float(json_data.get('forward_snr')),
                return_rssi=safe_float(json_data.get('return_rssi')),
                return_snr=safe_float(json_data.get('return_snr')),
                timestamp=json_data.get('timestamp', ''),
                status=json_data.get('status', 'Unknown')
            )
            
            self.tracker_data_map[target_id] = tracker_data
            self.tracker_ready_map[target_id] = True
    
    def calculate_current_quality(self) -> float:
        with self.lock:
            if len(self.dynamic_tracker_ids) < 2:
                return 0.0
            
            a_id, b_id = self.dynamic_tracker_ids
            if a_id not in self.tracker_data_map or b_id not in self.tracker_data_map:
                return 0.0
            
            tracker_a = self.tracker_data_map[a_id]
            tracker_b = self.tracker_data_map[b_id]
            
            rssi_avg = (tracker_a.forward_rssi + tracker_a.return_rssi + 
                       tracker_b.forward_rssi + tracker_b.return_rssi) / 4.0
            
            snr_avg = (tracker_a.forward_snr + tracker_a.return_snr + 
                      tracker_b.forward_snr + tracker_b.return_snr) / 4.0
            
            rssi_score = (rssi_avg + 100) / 80.0
            rssi_score = max(0.0, min(1.0, rssi_score))
            
            snr_score = (snr_avg + 20) / 35.0
            snr_score = max(0.0, min(1.0, snr_score))
            
            return 0.5 * rssi_score + 0.5 * snr_score


class KeyboardCommander:
    """键盘控制 Mixin"""
    
    HELP_MSG = """
══════════════════════════════════════════════════════════════
多无人机键盘控制台 - Position Control Mode
══════════════════════════════════════════════════════════════
🚁 飞行控制：
  SPACE  : 🚀 自动起飞（ARM → TAKEOFF 2.5m → OFFBOARD）
  L      : 🛬 降落并上锁
  H      : ⏸️  紧急悬停（锁定当前位置）

📡 扫描控制：
  F      : 启动信号扫描
  G      : 停止信号扫描

🔍 优化控制：
  S      : 开始信号优化
  P      : 暂停信号优化

✈️ 选择控制：
  1/2/3  : 选择单个无人机
  A      : 选择所有无人机

⚙️ 其他：
  Q      : 退出程序
  ?      : 显示此帮助
══════════════════════════════════════════════════════════════
    """
    
    def __init__(self):
        self.keyboard_thread = None
        self.terminal_settings = None
        self.selected_drones: Set[int] = set()
        self.optimization_enabled = True
        self.scan_enabled = False
        self.keyboard_running = True
        
    def init_keyboard_control(self, drone_ids: list):
        self.selected_drones = set(drone_ids)
        
        if sys.platform != 'win32':
            self.terminal_settings = termios.tcgetattr(sys.stdin)
        
        self.keyboard_thread = threading.Thread(
            target=self._keyboard_listener,
            daemon=True,
            name="KeyboardListener"
        )
        self.keyboard_thread.start()
        
        print(self.HELP_MSG)
        self._display_status()
        
    def _get_key_nonblocking(self):
        if sys.platform == 'win32':
            return None
        
        tty.setraw(sys.stdin.fileno())
        dr, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = None
        if dr:
            key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.terminal_settings)
        return key
    
    def _keyboard_listener(self):
        self.get_logger().info("键盘监听线程启动")
        
        while self.keyboard_running and rclpy.ok():
            try:
                key = self._get_key_nonblocking()
                if key:
                    self._handle_key(key)
            except Exception as e:
                self.get_logger().error(f"键盘监听异常: {e}")
            time.sleep(0.05)
        
        if sys.platform != 'win32' and self.terminal_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.terminal_settings)
        
        self.get_logger().info("键盘监听线程退出")
    
    def _handle_key(self, key: str):
        if not self.selected_drones:
            print("⚠️  请先选择无人机（按 1/2/3/A）")
            return
        
        if key == ' ':
            self._send_command_to_selected('ARM_TOGGLE')
            print(f"\n🚀 自动起飞: {sorted(self.selected_drones)}")
            self._display_status()
        
        elif key == 'l' or key == 'L':
            self._send_command_to_selected('LAND')
            print(f"\n🛬 降落命令: {sorted(self.selected_drones)}")
            self._display_status()
        
        elif key == 'h' or key == 'H':
            self._send_command_to_selected('HOLD')
            print(f"\n⏸️  紧急悬停: {sorted(self.selected_drones)}")
            self._display_status()
        
        elif key == 'f' or key == 'F':
            self.scan_enabled = True
            self._send_command_to_selected('START_SCAN')
            print(f"\n📡 信号扫描已启动: {sorted(self.selected_drones)}")
            self._display_status()
        
        elif key == 'g' or key == 'G':
            self.scan_enabled = False
            self._send_command_to_selected('STOP_SCAN')
            print(f"\n⏹️  信号扫描已停止")
            self._display_status()
        
        elif key == 's' or key == 'S':
            self.optimization_enabled = True
            print("\n🔍 信号优化已启动")
            self._display_status()
        
        elif key == 'p' or key == 'P':
            self.optimization_enabled = False
            # ✅ 位置控制：設定目標為當前位置（懸停）
            for drone_id in self.selected_drones:
                if drone_id in self.drone_states:
                    state = self.drone_states[drone_id]
                    with state.lock:
                        state.target_position = (
                            state.current_position.x,
                            state.current_position.y,
                            state.current_position.z
                        )
            print("\n⏸️  信号优化已暂停")
            self._display_status()
        
        elif key in ['1', '2', '3']:
            drone_id = int(key)
            if drone_id in self.drone_states:
                self.selected_drones = {drone_id}
                print(f"\n✓ 已选择 Drone {drone_id}")
                self._display_status()
            else:
                print(f"\n⚠️  Drone {drone_id} 不存在")
        
        elif key == 'a' or key == 'A':
            self.selected_drones = set(self.drone_states.keys())
            print(f"\n✓ 已选择所有无人机: {sorted(self.selected_drones)}")
            self._display_status()
        
        elif key == 'q' or key == 'Q':
            print("\n👋 正在退出...")
            self.keyboard_running = False
            self.running = False
            rclpy.shutdown()
        
        elif key == '?':
            print(self.HELP_MSG)
            self._display_status()
    
    def _send_command_to_selected(self, command: str):
        for drone_id in self.selected_drones:
            if drone_id in self.command_publishers:
                msg = String()
                msg.data = command
                self.command_publishers[drone_id].publish(msg)
    
    def _display_status(self):
        status_lines = []
        status_lines.append("\n" + "="*60)
        status_lines.append(f"🎯 位置控制模式 (Position Control)")
        status_lines.append(f"选中无人机: {sorted(self.selected_drones)}")
        status_lines.append(f"扫描状态: {'📡 运行中' if self.scan_enabled else '⏹️  已停止'}")
        status_lines.append(f"优化状态: {'🟢 运行中' if self.optimization_enabled else '🔴 已暂停'}")
        status_lines.append("─"*60)
        
        for drone_id in sorted(self.drone_states.keys()):
            state = self.drone_states[drone_id]
            with state.lock:
                selected = "👉" if drone_id in self.selected_drones else "  "
                quality = state.current_quality
                phase = state.scan_phase
                if state.target_position:
                    tx, ty, tz = state.target_position
                    target_str = f"T=({tx:.2f}, {ty:.2f}, {tz:.2f})"
                else:
                    target_str = "T=(未設定)"
                
            status_lines.append(
                f"{selected} Drone {drone_id}: Q={quality:.3f} | "
                f"Phase={phase} | {target_str}"
            )
        
        status_lines.append("="*60)
        print("\n".join(status_lines))


class MultiDronePositionOptimizer(Node, KeyboardCommander):
    """多无人机信号优化器（位置控制版本）"""
    
    # 常量配置
    MAX_MOVEMENTS = 10
    ALTITUDE_TARGET = 2.0
    PUBLISH_RATE = 10.0          # ✅ 降低到 10Hz（位置模式不需要高頻）
    ARRIVAL_THRESHOLD = 0.40     # ✅ 放寬到 0.40m（考慮 GPS 漂移）
    SCAN_WAIT_TIMEOUT = 120.0    # 掃描等待超時（秒）
    MOVE_TIMEOUT = 20.0          # ✅ 移動超時（秒）
    
    # 邊界
    SCAN_DISTANCE = 0.8          # ✅ 縮小到 0.8m（與邊界 1.2 保持 0.4m 緩衝）
    BOUNDS_X = (-1.2, 1.2)       # ✅ 縮小到 ±1.2m
    BOUNDS_Y = (-1.2, 1.2)
    BOUNDS_Z = (0.0, 3.0)
    
    def __init__(self):
        Node.__init__(self, 'multi_drone_position_optimizer')
        KeyboardCommander.__init__(self)
        
        self.declare_parameter('num_drones', 3)
        self.declare_parameter('drone_ids', [1, 2, 3])
        
        num_drones = self.get_parameter('num_drones').value
        drone_ids = self.get_parameter('drone_ids').value
        
        self.drone_states: Dict[int, DroneState] = {}
        for drone_id in drone_ids:
            self.drone_states[drone_id] = DroneState(drone_id=drone_id)
        
        self.reentrant_callback_group = ReentrantCallbackGroup()
        
        # 订阅 link_quality
        self.signal_subscribers = {}
        for drone_id in drone_ids:
            topic = f'/drone_{drone_id}/link_quality'
            self.signal_subscribers[drone_id] = self.create_subscription(
                String,
                topic,
                lambda msg, did=drone_id: self.link_quality_callback(msg, did),
                10,
                callback_group=self.reentrant_callback_group
            )
            self.get_logger().info(f"订阅 {topic}")
        
        # 订阅位置
        self.position_subscribers = {}
        for drone_id in drone_ids:
            topic = f'/drone_{drone_id}/fmu/out/vehicle_local_position'
            self.position_subscribers[drone_id] = self.create_subscription(
                VehicleLocalPosition,
                topic,
                lambda msg, did=drone_id: self.position_callback(msg, did),
                10,
                callback_group=self.reentrant_callback_group
            )
        
        # ✅ 發布位置指令（PoseStamped）
        self.position_publishers = {}
        for drone_id in drone_ids:
            topic = f'/drone_{drone_id}/offboard_position_cmd'
            self.position_publishers[drone_id] = self.create_publisher(
                PoseStamped,
                topic,
                10
            )
            self.get_logger().info(f"位置发布到 {topic}")
        
        # 发布命令
        self.command_publishers = {}
        for drone_id in drone_ids:
            topic = f'/drone_{drone_id}/command'
            self.command_publishers[drone_id] = self.create_publisher(
                String,
                topic,
                10
            )
        
        # 决策线程
        self.decision_threads = {}
        self.running = True
        for drone_id in drone_ids:
            thread = threading.Thread(
                target=self.decision_loop,
                args=(drone_id,),
                daemon=True,
                name=f"DecisionThread-Drone{drone_id}"
            )
            thread.start()
            self.decision_threads[drone_id] = thread
            self.get_logger().info(f"启动 Drone {drone_id} 决策线程")
        
        # ✅ 位置發布線程（10Hz）
        self.publish_threads = {}
        for drone_id in drone_ids:
            thread = threading.Thread(
                target=self.position_publish_loop,
                args=(drone_id,),
                daemon=True,
                name=f"PublishThread-Drone{drone_id}"
            )
            thread.start()
            self.publish_threads[drone_id] = thread
        
        self.init_keyboard_control(drone_ids)
        
        self.get_logger().info(f"🎯 位置控制優化器啟動（{num_drones} 架无人机）")
    
    def link_quality_callback(self, msg: String, drone_id: int):
        try:
            data = json.loads(msg.data)
            state = self.drone_states[drone_id]
            state.update_tracker_data(data)
            
            target_id = data.get('target_id', 'unknown')
            self.get_logger().debug(
                f"Drone {drone_id}: 收到 {target_id} 信号"
            )
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Drone {drone_id}: JSON 解析错误: {e}")
        except Exception as e:
            self.get_logger().error(f"Drone {drone_id}: 回调异常: {e}")
    
    def position_callback(self, msg: VehicleLocalPosition, drone_id: int):
        state = self.drone_states[drone_id]
        with state.lock:
            state.current_position = Position(x=msg.x, y=msg.y, z=msg.z)
            
            if state.takeoff_position is None or state.movement_count == 0:
                state.takeoff_position = Position(x=msg.x, y=msg.y, z=msg.z)
    
    def publish_position(self, drone_id: int, x: float, y: float, z: float):
        """✅ 發布位置指令（PoseStamped）"""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        
        # 保持當前航向（四元數設為 0）
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0  # 全零 = 保持當前航向
        
        self.position_publishers[drone_id].publish(pose)
    
    def decision_loop(self, drone_id: int):
        """決策循環（位置控制版本）"""
        self.get_logger().info(f"Drone {drone_id}: 決策線程啟動（位置控制模式）")
        state = self.drone_states[drone_id]
        
        while self.running:
            try:
                if not self.optimization_enabled:
                    time.sleep(0.5)
                    continue
                
                phase = state.scan_phase
                
                # ========== INIT ==========
                if phase == ScanPhase.INIT:
                    with state.lock:
                        if state.current_position.x == 0.0 and state.current_position.y == 0.0:
                            time.sleep(0.5)
                            continue
                        state.origin_xy = (state.current_position.x, state.current_position.y)
                        state.origin_z = state.current_position.z  # ✅ 記錄原點 Z
                        state.target_position = (
                            state.current_position.x,
                            state.current_position.y,
                            state.current_position.z
                        )
                        state.scan_scores = {}
                        state.scan_phase = ScanPhase.ORIGIN_SCAN
                    self.get_logger().info(
                        f"Drone {drone_id}: 🎯 原點鎖定 ({state.origin_xy[0]:.2f}, {state.origin_xy[1]:.2f})"
                    )
                
                # ========== ORIGIN_SCAN ==========
                elif phase == ScanPhase.ORIGIN_SCAN:
                    self._wait_and_scan(drone_id, 'origin', ScanPhase.MOVE_FRONT)
                
                # ========== MOVE_FRONT ==========
                elif phase == ScanPhase.MOVE_FRONT:
                    self._move_to_scan_point(drone_id, 'front', ScanPhase.FRONT_SCAN)
                
                # ========== FRONT_SCAN ==========
                elif phase == ScanPhase.FRONT_SCAN:
                    self._wait_and_scan(drone_id, 'front', ScanPhase.MOVE_BACK)
                
                # ========== MOVE_BACK ==========
                elif phase == ScanPhase.MOVE_BACK:
                    self._move_to_scan_point(drone_id, 'back', ScanPhase.BACK_SCAN)
                
                # ========== BACK_SCAN ==========
                elif phase == ScanPhase.BACK_SCAN:
                    self._wait_and_scan(drone_id, 'back', ScanPhase.MOVE_LEFT)
                
                # ========== MOVE_LEFT ==========
                elif phase == ScanPhase.MOVE_LEFT:
                    self._move_to_scan_point(drone_id, 'left', ScanPhase.LEFT_SCAN)
                
                # ========== LEFT_SCAN ==========
                elif phase == ScanPhase.LEFT_SCAN:
                    self._wait_and_scan(drone_id, 'left', ScanPhase.MOVE_RIGHT)
                
                # ========== MOVE_RIGHT ==========
                elif phase == ScanPhase.MOVE_RIGHT:
                    self._move_to_scan_point(drone_id, 'right', ScanPhase.RIGHT_SCAN)
                
                # ========== RIGHT_SCAN ==========
                elif phase == ScanPhase.RIGHT_SCAN:
                    self._wait_and_scan(drone_id, 'right', ScanPhase.CHOOSE_BEST)
                
                # ========== CHOOSE_BEST ==========
                elif phase == ScanPhase.CHOOSE_BEST:
                    self._select_best_point(drone_id)
                
                # ========== MOVE_TO_BEST ==========
                elif phase == ScanPhase.MOVE_TO_BEST:
                    self._move_to_best_point(drone_id)
                
                # ========== HOVERING ==========
                elif phase == ScanPhase.HOVERING:
                    if state.both_trackers_ready():
                        quality = state.calculate_current_quality()
                        self.get_logger().info(
                            f"Drone {drone_id}: 📍 懸停中 - 品質={quality:.3f}"
                        )
                        state.reset_tracker_flags()
                        state.current_quality = quality
                    time.sleep(1.0)
                
                # ========== DONE ==========
                elif phase == ScanPhase.DONE:
                    time.sleep(1.0)
                
                else:
                    time.sleep(0.5)
                
            except Exception as e:
                self.get_logger().error(f"Drone {drone_id}: 異常: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(1.0)
        
        self.get_logger().info(f"Drone {drone_id}: 決策線程退出")
    
    def _get_scan_point_offset(self, direction: str) -> Tuple[float, float]:
        d = self.SCAN_DISTANCE
        offsets = {
            'front': (d, 0.0),
            'back': (-d, 0.0),
            'left': (0.0, -d),
            'right': (0.0, d),
            'origin': (0.0, 0.0)
        }
        return offsets.get(direction, (0.0, 0.0))
    
    def _move_to_scan_point(self, drone_id: int, direction: str, next_phase: str):
        """✅ 位置控制：直接設定目標位置（改進版：穩定判定）"""
        state = self.drone_states[drone_id]
        
        with state.lock:
            if state.origin_xy is None:
                state.scan_phase = ScanPhase.INIT
                return
            
            origin_x, origin_y = state.origin_xy
            origin_z = state.origin_z
            current_x = state.current_position.x
            current_y = state.current_position.y
            
            dx, dy = self._get_scan_point_offset(direction)
            target_x = origin_x + dx
            target_y = origin_y + dy
            target_z = origin_z  # 保持原點高度
            
            # 邊界裁剪
            target_x = max(origin_x + self.BOUNDS_X[0], min(origin_x + self.BOUNDS_X[1], target_x))
            target_y = max(origin_y + self.BOUNDS_Y[0], min(origin_y + self.BOUNDS_Y[1], target_y))
            
            # ✅ 設定目標位置
            state.target_position = (target_x, target_y, target_z)
            
            dist = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
            
            # ✅ 改進的到達判定：距離穩定 3 次才算到達
            if dist < self.ARRIVAL_THRESHOLD:
                if abs(dist - state.last_distance) < 0.05:  # 距離變化 < 5cm
                    state.distance_stable_count += 1
                else:
                    state.distance_stable_count = 0
                
                if state.distance_stable_count >= 3:  # 連續 3 次穩定
                    state.is_moving = False
                    state.scan_phase = next_phase
                    state.movement_count += 1
                    state.distance_stable_count = 0
                    self.get_logger().info(
                        f"Drone {drone_id}: ✅ 到達 {direction} 掃描點（距離 {dist:.2f}m，穩定）"
                    )
                    return
            elif dist < self.ARRIVAL_THRESHOLD + 0.2:  # ✅ 距離停滯檢測
                if abs(dist - state.last_distance) < 0.05:
                    state.distance_stable_count += 1
                else:
                    state.distance_stable_count = 0
                
                if state.distance_stable_count >= 15:  # 3秒 = 15 * 0.2s
                    state.is_moving = False
                    state.scan_phase = next_phase
                    state.movement_count += 1
                    state.distance_stable_count = 0
                    self.get_logger().warning(
                        f"Drone {drone_id}: ⚠️ {direction} 距離停滯（{dist:.2f}m），視為到達"
                    )
                    return
            else:
                state.distance_stable_count = 0
            
            state.last_distance = dist
            
            if not state.is_moving:
                state.is_moving = True
                state.move_start_time = time.time()
                self.get_logger().info(
                    f"Drone {drone_id}: 🚀 移動到 {direction} (目標: {target_x:.2f}, {target_y:.2f}, 距離: {dist:.2f}m)"
                )
            
            elapsed = time.time() - state.move_start_time
            if elapsed > self.MOVE_TIMEOUT:
                state.is_moving = False
                state.scan_phase = next_phase
                state.movement_count += 1
                state.distance_stable_count = 0
                self.get_logger().warning(
                    f"Drone {drone_id}: ⚠️ 移動超時（{direction}，距離 {dist:.2f}m，耗時 {elapsed:.1f}s），強制進入 {direction} 掃描"
                )
                return
        
        time.sleep(0.2)
    
    def _wait_and_scan(self, drone_id: int, point_name: str, next_phase: str):
        state = self.drone_states[drone_id]
        
        with state.lock:
            # 確保目標位置正確
            if point_name == 'origin' and state.origin_xy is not None:
                state.target_position = (state.origin_xy[0], state.origin_xy[1], state.origin_z)
            
            if state.scan_start_time == 0.0:
                state.scan_start_time = time.time()
                self.get_logger().info(
                    f"Drone {drone_id}: ⏳ {point_name} 掃描中..."
                )
        
        if state.both_trackers_ready():
            quality = state.calculate_current_quality()
            state.scan_scores[point_name] = quality
            state.reset_tracker_flags()
            state.scan_start_time = 0.0
            state.scan_phase = next_phase
            self.get_logger().info(
                f"Drone {drone_id}: ✅ {point_name} 掃描完成，品質={quality:.3f}"
            )
        elif time.time() - state.scan_start_time > self.SCAN_WAIT_TIMEOUT:
            self.get_logger().warning(
                f"Drone {drone_id}: ⚠️ {point_name} 掃描超時"
            )
            state.scan_scores[point_name] = 0.0
            state.reset_tracker_flags()
            state.scan_start_time = 0.0
            state.scan_phase = next_phase
        else:
            time.sleep(0.5)
    
    def _select_best_point(self, drone_id: int):
        state = self.drone_states[drone_id]
        
        with state.lock:
            scores = state.scan_scores
            if not scores:
                state.scan_phase = ScanPhase.DONE
                return
            
            best_dir = max(scores, key=scores.get)
            best_score = scores[best_dir]
            state.best_direction = best_dir
            
            self.get_logger().info(
                f"Drone {drone_id}: 🏆 五點掃描結果: {scores}"
            )
            self.get_logger().info(
                f"Drone {drone_id}: 🎯 最佳方向: {best_dir} (品質={best_score:.3f})"
            )
            
            origin_x, origin_y = state.origin_xy
            origin_z = state.origin_z
            
            if best_dir == 'origin':
                state.target_position = (origin_x, origin_y, origin_z)
                state.scan_phase = ScanPhase.HOVERING
                return
            
            dx, dy = self._get_scan_point_offset(best_dir)
            target_x = max(origin_x + self.BOUNDS_X[0], min(origin_x + self.BOUNDS_X[1], origin_x + dx))
            target_y = max(origin_y + self.BOUNDS_Y[0], min(origin_y + self.BOUNDS_Y[1], origin_y + dy))
            
            state.target_position = (target_x, target_y, origin_z)
            state.is_moving = False
            state.scan_phase = ScanPhase.MOVE_TO_BEST
    
    def _move_to_best_point(self, drone_id: int):
        state = self.drone_states[drone_id]
        
        with state.lock:
            if state.target_position is None:
                state.scan_phase = ScanPhase.HOVERING
                return
            
            target_x, target_y, target_z = state.target_position
            current_x = state.current_position.x
            current_y = state.current_position.y
            
            dist = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
            
            if dist < self.ARRIVAL_THRESHOLD:
                state.is_moving = False
                state.scan_phase = ScanPhase.HOVERING
                self.get_logger().info(
                    f"Drone {drone_id}: ✅ 已到達最佳點 {state.best_direction}"
                )
                return
            
            if not state.is_moving:
                state.is_moving = True
                state.move_start_time = time.time()
            
            if time.time() - state.move_start_time > self.MOVE_TIMEOUT:
                state.is_moving = False
                state.scan_phase = ScanPhase.HOVERING
                self.get_logger().warning(
                    f"Drone {drone_id}: ⚠️ 移動超時，進入懸停"
                )
                return
        
        time.sleep(0.2)
    
    def position_publish_loop(self, drone_id: int):
        """✅ 位置發布循環（10Hz）"""
        rate = 1.0 / self.PUBLISH_RATE
        state = self.drone_states[drone_id]
        
        while self.running:
            try:
                with state.lock:
                    if state.target_position is not None:
                        tx, ty, tz = state.target_position
                    elif state.current_position is not None:
                        tx = state.current_position.x
                        ty = state.current_position.y
                        tz = state.current_position.z
                    else:
                        tx, ty, tz = 0.0, 0.0, 0.0
                
                self.publish_position(drone_id, tx, ty, tz)
                time.sleep(rate)
                
            except Exception as e:
                self.get_logger().error(f"Drone {drone_id}: 發布異常: {e}")
                time.sleep(rate)
    
    def destroy_node(self):
        self.get_logger().info("正在关闭位置控制优化器...")
        self.running = False
        self.keyboard_running = False
        
        for drone_id, thread in self.decision_threads.items():
            thread.join(timeout=2.0)
        for drone_id, thread in self.publish_threads.items():
            thread.join(timeout=2.0)
        if self.keyboard_thread:
            self.keyboard_thread.join(timeout=2.0)
        
        if sys.platform != 'win32' and self.terminal_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.terminal_settings)
        
        Node.destroy_node(self)


def main(args=None):
    rclpy.init(args=args)
    
    node = MultiDronePositionOptimizer()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("收到中断信号")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
