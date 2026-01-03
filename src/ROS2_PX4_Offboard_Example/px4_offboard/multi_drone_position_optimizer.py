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
import os
import csv
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String, Bool, ColorRGBA
from px4_msgs.msg import VehicleLocalPosition
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
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
    RETURN_TO_ORIGIN = "RETURN_TO_ORIGIN"  # ✅ 返回原點（中繼點）
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
    next_scan_target: Optional[str] = None   # ✅ 返回原點後的下一個目標（'left' 或 'right'）
    
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
        # ✅ 修正：允許全局快捷鍵（1/2/3/A/Q/?）在未選擇無人機時使用
        global_keys = ['1', '2', '3', 'a', 'A', 'q', 'Q', '?']
        
        if not self.selected_drones and key not in global_keys:
            print("⚠️  請先選擇無人機（按 1/2/3/A）")
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
            
            # ✅ 新增：發送掃描控制指令到 scan_control 話題
            if command == 'START_SCAN' and drone_id in self.scan_control_publishers:
                scan_msg = Bool()
                scan_msg.data = True
                self.scan_control_publishers[drone_id].publish(scan_msg)
            elif command == 'STOP_SCAN' and drone_id in self.scan_control_publishers:
                scan_msg = Bool()
                scan_msg.data = False
                self.scan_control_publishers[drone_id].publish(scan_msg)
    
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
    PUBLISH_RATE = 10.0          # ✅ 位置控制 10Hz（TrajectorySetpoint 建議頻率）
    ARRIVAL_THRESHOLD = 0.35     # ✅ 考慮 GPS 漂移，適度放寬（速度版本 0.30m）
    SCAN_WAIT_TIMEOUT = 120.0    # 掃描等待超時（秒）
    STABILIZE_WAIT = 3.0         # ✅ 穩定等待 3 秒（與速度版本統一）
    MOVE_TIMEOUT = 24.0          # ✅ 移動超時 24 秒（與速度版本 MOVE_DURATION*3 對齊）
    SCAN_VELOCITY = 0.15         # 掃描移動速度（用於日誌計算）
    
    # 邊界（與速度版本統一）
    SCAN_DISTANCE = 1.5          # ✅ 掃描距離 1.5m（與速度版本統一）
    BOUNDS_X = (-1.6, 1.6)       # ✅ X 軸邊界 ±1.6m（與速度版本統一）
    BOUNDS_Y = (-1.6, 1.6)       # ✅ Y 軸邊界 ±1.6m
    BOUNDS_Z = (0.0, 3.5)        # ✅ Z 軸邊界 3.5m（與速度版本統一）
    
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
        
        # ✅ 新增：掃描控制發布器（Bool）
        self.scan_control_publishers = {}
        for drone_id in drone_ids:
            topic = f'/drone_{drone_id}/scan_control'
            self.scan_control_publishers[drone_id] = self.create_publisher(
                Bool,
                topic,
                10
            )
            self.get_logger().info(f"掃描控制發布到 {topic}")
        
        # ✅ 新增：RViz2 可視化發布器
        self.origin_pubs = {}
        self.position_pubs = {}
        self.target_pubs = {}
        self.path_pubs = {}
        self.boundary_pubs = {}
        self.position_histories = {}  # 為每個無人機保存位置歷史
        self.path_messages = {}       # 為每個無人機保存軌跡信息
        
        for drone_id in drone_ids:
            self.origin_pubs[drone_id] = self.create_publisher(
                PoseStamped,
                f'/drone_{drone_id}/safety_origin',
                10
            )
            self.position_pubs[drone_id] = self.create_publisher(
                PoseStamped,
                f'/drone_{drone_id}/current_position',
                10
            )
            self.target_pubs[drone_id] = self.create_publisher(
                PoseStamped,
                f'/drone_{drone_id}/target_position',
                10
            )
            self.path_pubs[drone_id] = self.create_publisher(
                Path,
                f'/drone_{drone_id}/vehicle_path',
                10
            )
            self.boundary_pubs[drone_id] = self.create_publisher(
                MarkerArray,
                f'/drone_{drone_id}/safety_boundary',
                10
            )
            self.position_histories[drone_id] = deque(maxlen=50)
            self.path_messages[drone_id] = Path()
            self.path_messages[drone_id].header.frame_id = 'map'
        
        self.get_logger().info("RViz2 可視化發布器已初始化")
        
        # ✅ 新增：訊號歷史記錄器（每次飛行一個 CSV）
        self._init_signal_logger(drone_ids)
        
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
    
    def _init_signal_logger(self, drone_ids: list):
        """初始化訊號歷史記錄器（每架無人機一個 CSV 檔案）"""
        # 建立 signal_logs 目錄
        self.log_dir = os.path.expanduser("~/uav-core/signal_logs")
        os.makedirs(self.log_dir, exist_ok=True)
        
        # 產生飛行時間戳記（所有無人機共用）
        self.flight_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 為每架無人機建立 CSV 檔案
        self.signal_log_files = {}
        self.signal_log_writers = {}
        self.signal_log_lock = threading.Lock()
        
        # CSV 標頭（包含所有需要記錄的欄位）
        self.csv_headers = [
            "timestamp",           # 時間戳記
            "drone_id",            # 無人機 ID
            "scan_phase",          # 掃描階段
            "target_id",           # Tracker ID
            "status",              # 掃描狀態
            "forward_rssi",        # 前向 RSSI
            "forward_snr",         # 前向 SNR
            "return_rssi",         # 返向 RSSI
            "return_snr",          # 返向 SNR
            "quality_score",       # 綜合品質分數
            "pos_x",               # 位置 X (NED)
            "pos_y",               # 位置 Y (NED)
            "pos_z",               # 位置 Z (NED)
            "origin_x",            # 原點 X
            "origin_y",            # 原點 Y
        ]
        
        for drone_id in drone_ids:
            filename = f"flight_{self.flight_timestamp}_drone{drone_id}.csv"
            filepath = os.path.join(self.log_dir, filename)
            
            try:
                file = open(filepath, 'w', newline='', encoding='utf-8')
                writer = csv.DictWriter(file, fieldnames=self.csv_headers)
                writer.writeheader()
                
                self.signal_log_files[drone_id] = file
                self.signal_log_writers[drone_id] = writer
                
                self.get_logger().info(f"📊 Drone {drone_id} 訊號記錄: {filepath}")
            except Exception as e:
                self.get_logger().error(f"無法建立訊號記錄檔: {e}")
        
        self.get_logger().info(f"✅ 訊號歷史記錄器初始化完成，目錄: {self.log_dir}")
    
    def _log_signal_data(self, drone_id: int, data: dict):
        """記錄訊號數據到 CSV（線程安全）"""
        if drone_id not in self.signal_log_writers:
            return
        
        state = self.drone_states.get(drone_id)
        if state is None:
            return
        
        with self.signal_log_lock:
            try:
                # 計算品質分數
                tracker_data = None
                target_id = data.get('target_id', 'unknown')
                if target_id in state.tracker_data_map:
                    tracker_data = state.tracker_data_map[target_id]
                
                quality_score = tracker_data.quality_score if tracker_data else 0.0
                
                # 取得原點座標
                origin_x, origin_y = (0.0, 0.0)
                if state.origin_xy is not None:
                    origin_x, origin_y = state.origin_xy
                
                row = {
                    "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                    "drone_id": drone_id,
                    "scan_phase": state.scan_phase,
                    "target_id": target_id,
                    "status": data.get('status', 'Unknown'),
                    "forward_rssi": data.get('forward_rssi', ''),
                    "forward_snr": data.get('forward_snr', ''),
                    "return_rssi": data.get('return_rssi', ''),
                    "return_snr": data.get('return_snr', ''),
                    "quality_score": f"{quality_score:.4f}",
                    "pos_x": f"{state.current_position.x:.3f}",
                    "pos_y": f"{state.current_position.y:.3f}",
                    "pos_z": f"{state.current_position.z:.3f}",
                    "origin_x": f"{origin_x:.3f}",
                    "origin_y": f"{origin_y:.3f}",
                }
                
                self.signal_log_writers[drone_id].writerow(row)
                self.signal_log_files[drone_id].flush()  # 即時寫入磁碟
                
            except Exception as e:
                self.get_logger().error(f"訊號記錄失敗: {e}")
    
    def _close_signal_loggers(self):
        """關閉所有訊號記錄檔案"""
        with self.signal_log_lock:
            for drone_id, file in self.signal_log_files.items():
                try:
                    file.close()
                    self.get_logger().info(f"📊 Drone {drone_id} 訊號記錄已關閉")
                except Exception as e:
                    self.get_logger().error(f"關閉訊號記錄檔失敗: {e}")
            self.signal_log_files.clear()
            self.signal_log_writers.clear()

    
    def link_quality_callback(self, msg: String, drone_id: int):
        try:
            data = json.loads(msg.data)
            state = self.drone_states[drone_id]
            state.update_tracker_data(data)
            
            # ✅ 記錄訊號數據到 CSV（每次收到都記錄）
            self._log_signal_data(drone_id, data)
            
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
        
        # ✅ 修正：四元數改為單位四元數 (0,0,0,1)，表示保持當前航向（而非無效的全零）
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0  # ✅ 單位四元數
        
        self.position_publishers[drone_id].publish(pose)
        
        # ✅ 新增：發布 RViz2 可視化
        self._publish_rviz_visualization(drone_id, x, y, z)
    
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
                    self._wait_and_scan(drone_id, 'back', ScanPhase.RETURN_TO_ORIGIN, next_target='left')
                
                # ========== RETURN_TO_ORIGIN ==========
                elif phase == ScanPhase.RETURN_TO_ORIGIN:
                    self._move_to_origin(drone_id)
                
                # ========== MOVE_LEFT ==========
                elif phase == ScanPhase.MOVE_LEFT:
                    self._move_to_scan_point(drone_id, 'left', ScanPhase.LEFT_SCAN)
                
                # ========== LEFT_SCAN ==========
                elif phase == ScanPhase.LEFT_SCAN:
                    self._wait_and_scan(drone_id, 'left', ScanPhase.RETURN_TO_ORIGIN, next_target='right')
                
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
    
    def _move_to_origin(self, drone_id: int):
        """
        返回原點（位置控制版本）
        
        邏輯：
        - 根據 next_scan_target 判斷返回後的下一個目標
        - 直接設定目標位置為原點
        - 到達原點後進入對應的 MOVE_* 階段
        """
        state = self.drone_states[drone_id]
        
        with state.lock:
            if state.origin_xy is None:
                state.scan_phase = ScanPhase.INIT
                return
            
            origin_x, origin_y = state.origin_xy
            origin_z = state.origin_z
            current_x = state.current_position.x
            current_y = state.current_position.y
            
            # 設定目標位置為原點
            state.target_position = (origin_x, origin_y, origin_z)
            
            # 計算到原點的距離
            dist = math.sqrt((origin_x - current_x)**2 + (origin_y - current_y)**2)
            
            # ✅ 到達原點判定（GPS 考慮 0.35m 容差）
            if dist < self.ARRIVAL_THRESHOLD:
                if abs(dist - state.last_distance) < 0.05:
                    state.distance_stable_count += 1
                else:
                    state.distance_stable_count = 0
                
                if state.distance_stable_count >= 3:
                    # 根據 next_scan_target 決定下一階段
                    if state.next_scan_target == 'left':
                        state.scan_phase = ScanPhase.MOVE_LEFT
                        next_desc = "left"
                    elif state.next_scan_target == 'right':
                        state.scan_phase = ScanPhase.MOVE_RIGHT
                        next_desc = "right"
                    else:
                        # 異常情況：已完成所有掃描
                        state.scan_phase = ScanPhase.CHOOSE_BEST
                        next_desc = "CHOOSE_BEST"
                    
                    state.next_scan_target = None
                    state.is_moving = False
                    state.distance_stable_count = 0
                    state.target_position = None
                    self.get_logger().info(
                        f"Drone {drone_id}: ✅ 已返回原點，準備移動到 {next_desc}"
                    )
                    return
            else:
                state.distance_stable_count = 0
            
            state.last_distance = dist
            
            # 開始或繼續移動到原點
            if not state.is_moving:
                state.move_start_time = time.time()
                state.is_moving = True
                self.get_logger().info(
                    f"Drone {drone_id}: 🔄 返回原點 (距離: {dist:.2f}m)"
                )
            
            # 超時檢查
            elapsed = time.time() - state.move_start_time
            if elapsed > self.MOVE_TIMEOUT:
                self.get_logger().warning(
                    f"Drone {drone_id}: ⚠️ 返回原點超時（{elapsed:.1f}s），強制進入下一階段"
                )
                state.scan_phase = ScanPhase.CHOOSE_BEST
                state.target_position = None
                return
        
        time.sleep(0.2)
    
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
    
    def _wait_and_scan(self, drone_id: int, point_name: str, next_phase: str, next_target: Optional[str] = None):
        """✅ 改進版：包含穩定等待邏輯
        
        Args:
            drone_id: 無人機 ID
            point_name: 掃描點名稱
            next_phase: 掃描完成後的下一階段
            next_target: ✅ 如果 next_phase 是 RETURN_TO_ORIGIN，此參數指定返回後的目標
        """
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
        
        elapsed = time.time() - state.scan_start_time
        
        # ✅ 新增：穩定等待邏輯（先等待 STABILIZE_WAIT 秒，再等待 tracker ready）
        if elapsed < self.STABILIZE_WAIT:
            # 穩定等待中，重置 tracker flags 以清除舊數據
            state.reset_tracker_flags()
            time.sleep(0.1)
            return
        
        if state.both_trackers_ready():
            quality = state.calculate_current_quality()
            state.scan_scores[point_name] = quality
            state.reset_tracker_flags()
            state.scan_start_time = 0.0
            state.scan_phase = next_phase
            
            # ✅ 如果下一階段是返回原點，設定 next_scan_target
            if next_phase == ScanPhase.RETURN_TO_ORIGIN and next_target:
                state.next_scan_target = next_target
                self.get_logger().info(
                    f"Drone {drone_id}: ✅ {point_name} 掃描完成，品質={quality:.3f}，準備返回原點"
                )
            else:
                self.get_logger().info(
                    f"Drone {drone_id}: ✅ {point_name} 掃描完成，品質={quality:.3f}"
                )
        elif elapsed > self.SCAN_WAIT_TIMEOUT:
            self.get_logger().warning(
                f"Drone {drone_id}: ⚠️ {point_name} 掃描超時（{elapsed:.1f}s）"
            )
            state.scan_scores[point_name] = 0.0
            state.reset_tracker_flags()
            state.scan_start_time = 0.0
            state.scan_phase = next_phase
            
            # ✅ 超時時也要設定 next_scan_target
            if next_phase == ScanPhase.RETURN_TO_ORIGIN and next_target:
                state.next_scan_target = next_target
        else:
            time.sleep(0.1)
    
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
        """✅ 位置發布循環（10Hz，TrajectorySetpoint 建議頻率）"""
        rate = 1.0 / self.PUBLISH_RATE  # 10Hz = 0.1s
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
                    
                    # ✅ 新增：記錄位置歷史（用於 RViz2 軌跡）
                    self.position_histories[drone_id].append({
                        'x': state.current_position.x,
                        'y': state.current_position.y,
                        'z': state.current_position.z
                    })
                
                self.publish_position(drone_id, tx, ty, tz)
                time.sleep(rate)
                
            except Exception as e:
                self.get_logger().error(f"Drone {drone_id}: 發布異常: {e}")
                time.sleep(rate)
    
    def _publish_rviz_visualization(self, drone_id: int, tx: float, ty: float, tz: float):
        """✅ 新增：發布 RViz2 可視化數據（NED → ENU 轉換）"""
        if drone_id not in self.drone_states:
            return
        
        state = self.drone_states[drone_id]
        now = self.get_clock().now().to_msg()
        frame_id = 'map'
        
        # NED → ENU 轉換
        def ned_to_enu(ned_x, ned_y, ned_z):
            enu_x = ned_y      # East = NED East
            enu_y = ned_x      # North = NED North
            enu_z = -ned_z     # Up = -NED Down
            return enu_x, enu_y, enu_z
        
        # 原點
        if state.origin_xy is not None:
            origin_pose = PoseStamped()
            origin_pose.header.stamp = now
            origin_pose.header.frame_id = frame_id
            o_x, o_y, o_z = ned_to_enu(state.origin_xy[0], state.origin_xy[1], state.origin_z)
            origin_pose.pose.position.x = o_x
            origin_pose.pose.position.y = o_y
            origin_pose.pose.position.z = o_z
            origin_pose.pose.orientation.w = 1.0
            self.origin_pubs[drone_id].publish(origin_pose)
        
        # 當前位置
        current_pose = PoseStamped()
        current_pose.header.stamp = now
        current_pose.header.frame_id = frame_id
        c_x, c_y, c_z = ned_to_enu(state.current_position.x, state.current_position.y, state.current_position.z)
        current_pose.pose.position.x = c_x
        current_pose.pose.position.y = c_y
        current_pose.pose.position.z = c_z
        current_pose.pose.orientation.w = 1.0
        self.position_pubs[drone_id].publish(current_pose)
        
        # 目標位置
        if state.target_position is not None:
            target_pose = PoseStamped()
            target_pose.header.stamp = now
            target_pose.header.frame_id = frame_id
            target_x, target_y, target_z = state.target_position
            t_x, t_y, t_z = ned_to_enu(target_x, target_y, target_z)
            target_pose.pose.position.x = t_x
            target_pose.pose.position.y = t_y
            target_pose.pose.position.z = t_z
            target_pose.pose.orientation.w = 1.0
            self.target_pubs[drone_id].publish(target_pose)
        
        # 軌跡
        self.path_messages[drone_id].header = current_pose.header
        self.path_messages[drone_id].poses.append(current_pose)
        self.path_pubs[drone_id].publish(self.path_messages[drone_id])
        
        # 邊界框
        marker_array = MarkerArray()
        boundary_marker = Marker()
        boundary_marker.header.stamp = now
        boundary_marker.header.frame_id = frame_id
        boundary_marker.id = 0
        boundary_marker.type = Marker.CUBE
        boundary_marker.action = Marker.ADD
        
        if state.origin_xy is not None:
            b_x, b_y, b_z = ned_to_enu(state.origin_xy[0], state.origin_xy[1], state.origin_z - 1.5)
        else:
            b_x, b_y, b_z = 0.0, 0.0, 0.0
        
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
        
        self.boundary_pubs[drone_id].publish(marker_array)
    
    def destroy_node(self):
        self.get_logger().info("正在关闭位置控制优化器...")
        self.running = False
        self.keyboard_running = False
        
        # ✅ 關閉訊號記錄檔案
        self._close_signal_loggers()
        
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
