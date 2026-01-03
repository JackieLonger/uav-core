#!/usr/bin/env python3
"""
Multi-Drone Signal Optimizer - 多无人机协同信号优化器

架构：
- 笔记本运行此节点（集中决策）
- 每个 Jetson 运行 fast_scan_node.py（扫描并发布到 /drone_N/link_quality）
- 每个 Jetson 运行 velocity_control.py（接收 /drone_N/offboard_velocity_cmd）
- 多线程异步处理每个无人机的决策（不阻塞）
- 键盘控制：统一控制多架无人机（ARM, TAKEOFF, OFFBOARD, LAND, HOLD）

策略：
1. 优先上升到 1.5m
2. RSSI + SNR 综合评分（各占50%）
3. 第n次 > 第n-1次 → 继续方向
4. 信号变差 → XY随机小幅搜索
5. 最多 5 次移动
6. 速度限制 0.3 m/s

键盘控制：
- SPACE: ARM/DISARM 所有无人机
- T: 起飞到 2.5m
- O: 进入 Offboard 模式
- L: 降落
- H: 紧急悬停
- S: 开始优化
- P: 暂停优化
- 1/2/3: 选择控制单个无人机
- A: 选择所有无人机
- Q: 退出程序
"""

import sys
import os
import csv
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from px4_msgs.msg import VehicleLocalPosition
import threading
import json
import time
import math
import random
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
            # 确保 RSSI 和 SNR 是浮点数（可能从 JSON 来是字符串）
            forward_rssi = float(self.forward_rssi) if self.forward_rssi else 0.0
            return_rssi = float(self.return_rssi) if self.return_rssi else 0.0
            forward_snr = float(self.forward_snr) if self.forward_snr else 0.0
            return_snr = float(self.return_snr) if self.return_snr else 0.0
            
            # RSSI 归一化 (-120 to -30 dBm → 0 to 1)
            rssi_avg = (forward_rssi + return_rssi) / 2.0
            rssi_score = (rssi_avg + 120) / 90.0
            rssi_score = max(0.0, min(1.0, rssi_score))
            
            # SNR 归一化 (-20 to 15 dB → 0 to 1)
            snr_avg = (forward_snr + return_snr) / 2.0
            snr_score = (snr_avg + 20) / 35.0
            snr_score = max(0.0, min(1.0, snr_score))
            
            # 综合评分 (RSSI 50% + SNR 50%)
            return 0.5 * rssi_score + 0.5 * snr_score
        except (ValueError, TypeError):
            # 如果数据无效，返回 0
            return 0.0


@dataclass
class Position:
    """3D位置"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    
    def distance_to(self, other: 'Position') -> float:
        """计算到另一位置的距离"""
        return math.sqrt(
            (self.x - other.x)**2 + 
            (self.y - other.y)**2 + 
            (self.z - other.z)**2
        )
    
    def __sub__(self, other: 'Position') -> 'Position':
        """位置差"""
        return Position(
            self.x - other.x,
            self.y - other.y,
            self.z - other.z
        )
    
    def normalized(self) -> 'Position':
        """归一化向量"""
        length = math.sqrt(self.x**2 + self.y**2 + self.z**2)
        if length < 0.001:
            return Position(0, 0, 0)
        return Position(
            self.x / length,
            self.y / length,
            self.z / length
        )


# 掃描序列狀態枚舉
class ScanPhase:
    INIT = "INIT"                    # 初始化，等待位置數據
    ORIGIN_SCAN = "ORIGIN_SCAN"      # 原點掃描
    MOVE_FRONT = "MOVE_FRONT"        # 移動到前方
    FRONT_SCAN = "FRONT_SCAN"        # 前方掃描
    MOVE_BACK = "MOVE_BACK"          # 移動到後方
    BACK_SCAN = "BACK_SCAN"          # 後方掃描
    RETURN_TO_ORIGIN = "RETURN_TO_ORIGIN"  # ✅ 返回原點（中繼點）
    MOVE_LEFT = "MOVE_LEFT"          # 移動到左方
    LEFT_SCAN = "LEFT_SCAN"          # 左方掃描
    MOVE_RIGHT = "MOVE_RIGHT"        # 移動到右方
    RIGHT_SCAN = "RIGHT_SCAN"        # 右方掃描
    CHOOSE_BEST = "CHOOSE_BEST"      # 選擇最佳點
    MOVE_TO_BEST = "MOVE_TO_BEST"    # 移動到最佳點
    HOVERING = "HOVERING"            # 懸停監控（已到達最佳點）
    DONE = "DONE"                    # 完成


@dataclass
class DroneState:
    """单个无人机的状态（线程安全）"""
    drone_id: int
    
    # Tracker 数据 (动态绑定)
    dynamic_tracker_ids: list = field(default_factory=list)   # 最多两个
    tracker_data_map: Dict[str, TrackerData] = field(default_factory=dict)
    tracker_ready_map: Dict[str, bool] = field(default_factory=dict)
    
    # 位置信息
    current_position: Position = field(default_factory=Position)
    takeoff_position: Optional[Position] = None
    previous_position: Optional[Position] = None
    
    # 决策状态
    previous_quality: float = 0.0
    current_quality: float = 0.0
    movement_count: int = 0
    
    # 线程锁
    lock: threading.Lock = field(default_factory=threading.Lock)
    
    # 速度指令
    current_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    
    # 质量历史
    quality_history: deque = field(default_factory=lambda: deque(maxlen=10))
    
    # ========== 掃描序列狀態 ==========
    scan_phase: str = ScanPhase.INIT           # 當前掃描階段
    origin_xy: Optional[Tuple[float, float]] = None  # 原點 XY 座標
    scan_scores: Dict[str, float] = field(default_factory=dict)  # 各點品質分數
    best_direction: Optional[str] = None       # 最佳方向 (front/back/left/right/origin)
    target_xy: Optional[Tuple[float, float]] = None  # 目標位置（移動到最佳點時使用）
    move_start_time: float = 0.0               # 移動開始時間
    is_moving: bool = False                    # 是否正在移動中
    scan_start_time: float = 0.0               # 掃描開始時間（用於超時保護）
    current_cycle_id: int = 0                  # 當前掃描輪次 ID
    last_distance: float = float('inf')        # ✅ 上次距離（穩定判定）
    distance_stable_count: int = 0             # ✅ 距離穩定計數
    next_scan_target: Optional[str] = None     # ✅ 返回原點後的下一個目標（'left' 或 'right'）
    
    def cycle_complete(self) -> bool:
        """檢查當前輪次五點是否都已掃描"""
        required = {'origin', 'front', 'back', 'left', 'right'}
        return required.issubset(self.scan_scores.keys())
    
    def start_new_cycle(self):
        """開始新的掃描輪次"""
        self.current_cycle_id += 1
        self.scan_scores = {}  # 清空上一輪分數
        self.reset_tracker_flags()
    
    def both_trackers_ready(self) -> bool:
        """两个信号都到了"""
        with self.lock:
            if len(self.dynamic_tracker_ids) < 2:
                return False
            # 两个都在并且 ready
            return all(self.tracker_ready_map.get(tid, False) for tid in self.dynamic_tracker_ids)
    
    def reset_tracker_flags(self):
        """决策后重置，等待下一轮扫描"""
        with self.lock:
            for tid in self.dynamic_tracker_ids:
                self.tracker_ready_map[tid] = False
    
    def update_tracker_data(self, json_data: dict):
        """更新 tracker 数据（线程安全）"""
        with self.lock:
            # 跳过失败或超時的掃描
            status = json_data.get('status', 'Unknown')
            if status not in ['Success']:
                return

            target_id = json_data['target_id']
            
            # 若尚未收集满两个 tracker，加入新 ID
            if target_id not in self.dynamic_tracker_ids and len(self.dynamic_tracker_ids) < 2:
                self.dynamic_tracker_ids.append(target_id)
            
            # 若收到不在本无人机绑定范围的第三种 ID，直接忽略
            if target_id not in self.dynamic_tracker_ids:
                return
            
            # 安全地轉換 RSSI/SNR 為浮點數（處理字符串、空值、N/A）
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
        """計算當前綜合質量（平均兩個 Tracker）"""
        with self.lock:
            if len(self.dynamic_tracker_ids) < 2:
                return 0.0
            
            a_id, b_id = self.dynamic_tracker_ids
            if a_id not in self.tracker_data_map or b_id not in self.tracker_data_map:
                return 0.0
            
            # 獲取兩個 Tracker 的原始信號值
            tracker_a = self.tracker_data_map[a_id]
            tracker_b = self.tracker_data_map[b_id]
            
            # 平均 RSSI (來回平均)
            rssi_avg = (tracker_a.forward_rssi + tracker_a.return_rssi + 
                       tracker_b.forward_rssi + tracker_b.return_rssi) / 4.0
            
            # 平均 SNR (來回平均)
            snr_avg = (tracker_a.forward_snr + tracker_a.return_snr + 
                      tracker_b.forward_snr + tracker_b.return_snr) / 4.0
            
            # 歸一化 RSSI (-100 到 -20 dBm → 0 到 1)
            rssi_score = (rssi_avg + 100) / 80.0
            rssi_score = max(0.0, min(1.0, rssi_score))
            
            # 歸一化 SNR (-20 到 15 dB → 0 到 1)
            snr_score = (snr_avg + 20) / 35.0
            snr_score = max(0.0, min(1.0, snr_score))
            
            # 加權平均 (RSSI 50%, SNR 50%)
            quality = 0.5 * rssi_score + 0.5 * snr_score
            
            return quality


class KeyboardCommander:
    """键盘控制 Mixin（多无人机统一控制）"""
    
    HELP_MSG = """
══════════════════════════════════════════════════════════════
多无人机键盘控制台 - Multi-Drone Keyboard Commander
══════════════════════════════════════════════════════════════
🚁 飞行控制：
  SPACE  : 🚀 自动起飞（ARM → TAKEOFF 2.5m → OFFBOARD）
  L      : 🛬 降落并上锁
  H      : ⏸️  紧急悬停（立即停止移动）

📡 扫描控制：
  F      : 启动信号扫描（Fast Scan）
  G      : 停止信号扫描

🔍 优化控制：
  S      : 开始信号优化
  P      : 暂停信号优化

✈️ 选择控制：
  1/2/3  : 选择单个无人机（Drone 1/2/3）
  A      : 选择所有无人机

⚙️ 其他：
  Q      : 退出程序
  ?      : 显示此帮助
══════════════════════════════════════════════════════════════
    """
    
    def __init__(self):
        """初始化键盘控制器"""
        self.keyboard_thread = None
        self.terminal_settings = None
        self.selected_drones: Set[int] = set()  # 当前选中的无人机 ID
        self.optimization_enabled = True  # 优化默认启用
        self.scan_enabled = False  # 扫描是否启用（需手动启动）
        self.keyboard_running = True
        
    def init_keyboard_control(self, drone_ids: list):
        """初始化键盘控制（在 ROS2 node 初始化后调用）"""
        # 默认选择所有无人机
        self.selected_drones = set(drone_ids)
        
        # ✅ 檢查是否在互動式終端
        if not sys.stdin.isatty():
            self.get_logger().warn("⚠️  未在互動式終端執行，鍵盤控制已禁用")
            self.keyboard_running = False
            return
        
        # 保存终端设置
        if sys.platform != 'win32':
            try:
                self.terminal_settings = termios.tcgetattr(sys.stdin)
            except termios.error as e:
                self.get_logger().error(f"❌ 無法獲取終端設定: {e}")
                self.keyboard_running = False
                return
        
        # 启动键盘监听线程
        self.keyboard_thread = threading.Thread(
            target=self._keyboard_listener,
            daemon=True,
            name="KeyboardListener"
        )
        self.keyboard_thread.start()
        
        # ❌ 移除定时刷新，改为按键触发刷新
        # self.status_timer = self.create_timer(2.0, self._display_status)
        
        # 显示帮助信息
        print(self.HELP_MSG)
        self._display_status()
        
    def _get_key_nonblocking(self):
        """非阻塞式读取单个按键（Linux/Unix）"""
        if sys.platform == 'win32':
            # Windows 暂不支持
            return None
        
        try:
            tty.setraw(sys.stdin.fileno())
            dr, _, _ = select.select([sys.stdin], [], [], 0.05)  # 减少等待时间
            key = None
            if dr:
                key = sys.stdin.read(1)
            return key
        finally:
            # 无论如何都要恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.terminal_settings)
    
    def _keyboard_listener(self):
        """键盘监听线程"""
        self.get_logger().info("键盘监听线程启动")
        
        while self.keyboard_running and rclpy.ok():
            try:
                key = self._get_key_nonblocking()
                if key:
                    self._handle_key(key)
            except Exception as e:
                self.get_logger().error(f"键盘监听异常: {e}")
            time.sleep(0.05)
        
        # 恢复终端设置
        if sys.platform != 'win32' and self.terminal_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.terminal_settings)
        
        self.get_logger().info("键盘监听线程退出")
    
    def _handle_key(self, key: str):
        """处理按键事件"""
        # ✅ 优先处理 "全局" 按键（即使未选择无人机也可执行）
        if key == '?':
            print(self.HELP_MSG)
            self._display_status()
            return
        
        # 选择单个无人机（全局快捷键）
        if key in ['1', '2', '3']:
            drone_id = int(key)
            if drone_id in self.drone_states:
                self.selected_drones = {drone_id}
                print(f"\n✓ 已选择 Drone {drone_id}")
                self._display_status()
            else:
                print(f"\n⚠️  Drone {drone_id} 不存在")
            return
        
        # 选择所有无人机（全局快捷键）
        if key == 'a' or key == 'A':
            self.selected_drones = set(self.drone_states.keys())
            print(f"\n✓ 已选择所有无人机: {sorted(self.selected_drones)}")
            self._display_status()
            return
        
        # 退出（全局快捷键）
        if key == 'q' or key == 'Q':
            print("\n👋 正在退出...")
            self.keyboard_running = False
            self.running = False
            # ✅ 改為設置標誌，讓主線程負責 shutdown
            return
        
        # ✅ 从这里开始，所有命令都需要选择无人机
        if not self.selected_drones:
            print("⚠️  请先选择无人机（按 1/2/3/A）")
            return
        
        # SPACE: 自动起飞流程 ARM → TAKEOFF → OFFBOARD
        if key == ' ':
            self._send_command_to_selected('ARM_TOGGLE')
            print(f"\n🚀 自动起飞: ARM → TAKEOFF(2.5m) → OFFBOARD: {sorted(self.selected_drones)}")
            self._display_status()
        
        # L: 降落
        elif key == 'l' or key == 'L':
            self._send_command_to_selected('LAND')
            print(f"\n🛬 降落命令: {sorted(self.selected_drones)}")
            self._display_status()
        
        # H: 紧急悬停
        elif key == 'h' or key == 'H':
            self._send_command_to_selected('HOLD')
            print(f"\n⏸️  紧急悬停: {sorted(self.selected_drones)}")
            self._display_status()
        
        # 启动扫描
        elif key == 'f' or key == 'F':
            self.scan_enabled = True
            self._send_command_to_selected('START_SCAN')
            self._send_scan_control(True)  # ✅ 發送 Bool 到 fast_scan_node
            print(f"\n📡 信号扫描已启动: {sorted(self.selected_drones)}")
            self._display_status()
        
        # 停止扫描
        elif key == 'g' or key == 'G':
            self.scan_enabled = False
            self._send_command_to_selected('STOP_SCAN')
            self._send_scan_control(False)  # ✅ 發送 Bool 到 fast_scan_node
            print(f"\n⏹️  信号扫描已停止: {sorted(self.selected_drones)}")
            self._display_status()
        
        # 开始优化
        elif key == 's' or key == 'S':
            self.optimization_enabled = True
            print("\n🔍 信号优化已启动")
            self._display_status()
        
        # 暂停优化
        elif key == 'p' or key == 'P':
            self.optimization_enabled = False
            # 停止所有速度指令
            for drone_id in self.selected_drones:
                if drone_id in self.drone_states:
                    with self.drone_states[drone_id].lock:
                        self.drone_states[drone_id].current_velocity = (0.0, 0.0, 0.0)
            print("\n⏸️  信号优化已暂停")
            self._display_status()
    
    def _send_command_to_selected(self, command: str):
        """向选中的无人机发送命令"""
        for drone_id in self.selected_drones:
            if drone_id in self.command_publishers:
                msg = String()
                msg.data = command
                self.command_publishers[drone_id].publish(msg)
    
    def _send_scan_control(self, enabled: bool):
        """✅ 向选中的无人机发送扫描控制命令（Bool 類型，與 fast_scan_node.py 配合）"""
        for drone_id in self.selected_drones:
            if drone_id in self.scan_control_publishers:
                msg = Bool()
                msg.data = enabled
                self.scan_control_publishers[drone_id].publish(msg)
                self.get_logger().info(f"Drone {drone_id}: 扫描控制 = {enabled}")
    
    def _display_status(self):
        """显示当前状态（定时刷新）"""
        status_lines = []
        status_lines.append("\n" + "="*60)
        status_lines.append(f"选中无人机: {sorted(self.selected_drones)}")
        status_lines.append(f"扫描状态: {'📡 运行中' if self.scan_enabled else '⏹️  已停止'}")
        status_lines.append(f"优化状态: {'🟢 运行中' if self.optimization_enabled else '🔴 已暂停'}")
        status_lines.append("─"*60)
        
        for drone_id in sorted(self.drone_states.keys()):
            state = self.drone_states[drone_id]
            with state.lock:
                selected = "👉" if drone_id in self.selected_drones else "  "
                quality = state.current_quality
                moves = state.movement_count
                vx, vy, vz = state.current_velocity
                
            status_lines.append(
                f"{selected} Drone {drone_id}: Q={quality:.3f} | "
                f"Moves={moves}/{self.MAX_MOVEMENTS} | "
                f"V=({vx:.2f}, {vy:.2f}, {vz:.2f})"
            )
        
        status_lines.append("="*60)
        print("\n".join(status_lines))


class MultiDroneSignalOptimizer(Node, KeyboardCommander):
    """多无人机信号优化器（多线程異步 + 鍵盤控制）"""
    
    # 常量配置
    MAX_MOVEMENTS = 10           # 最大移動次數（包含邊界掃描）
    MAX_VELOCITY = 0.2           # m/s（降低速度，更精確）
    SCAN_VELOCITY = 0.15         # m/s（掃描移動速度，更慢更穩）
    ALTITUDE_TARGET = 2.5        # ✅ 固定高度 2.5m（XY平面掃描）
    ALTITUDE_THRESHOLD = 1.5     # 舊參數保留相容
    PUBLISH_RATE = 100.0         # Hz (100Hz for PX4 Offboard)
    MOVE_DURATION = 8.0          # ✅ 每次移動持續時間（從 4s 增加到 8s）
    SCAN_WAIT_TIMEOUT = 120.0    # 掃描等待超時（秒）
    STABILIZE_WAIT = 3.0         # ✅ 到達掃描點後的穩定等待時間（3秒，確保信號量測穩定）
    ARRIVAL_THRESHOLD = 0.30     # ✅ 到達判定距離（收緊到 0.30m 提高精度）
    
    # 邊界（相對於 origin_xy）- 五點掃描：原點 + 前後左右各 1.5m
    SCAN_DISTANCE = 1.5          # ✅ 掃描點距離（1.5m 半徑）
    BOUNDS_X = (-1.6, 1.6)       # ✅ X 軸邊界（±1.6m，比掃描距離多 0.1m 緩衝）
    BOUNDS_Y = (-1.6, 1.6)       # ✅ Y 軸邊界
    BOUNDS_Z = (0.0, 3.5)        # Z 軸邊界（只能上升 0~3.5m）
    
    def __init__(self):
        # 初始化 Node
        Node.__init__(self, 'multi_drone_signal_optimizer')
        # 初始化 KeyboardCommander
        KeyboardCommander.__init__(self)
        
        # 声明参数
        self.declare_parameter('num_drones', 3)
        self.declare_parameter('drone_ids', [1, 2, 3])
        
        num_drones = self.get_parameter('num_drones').value
        drone_ids = self.get_parameter('drone_ids').value
        
        # 初始化每个无人机的状态
        self.drone_states: Dict[int, DroneState] = {}
        for drone_id in drone_ids:
            self.drone_states[drone_id] = DroneState(drone_id=drone_id)
        
        # 回调组（允许并行处理）
        self.reentrant_callback_group = ReentrantCallbackGroup()
        
        # 订阅每个无人机的 link_quality
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
        
        # 订阅每个无人机的位置
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
        
        # 发布速度指令
        self.velocity_publishers = {}
        for drone_id in drone_ids:
            topic = f'/drone_{drone_id}/offboard_velocity_cmd'
            self.velocity_publishers[drone_id] = self.create_publisher(
                Twist,
                topic,
                10
            )
            self.get_logger().info(f"发布到 {topic}")
        
        # 发布命令（键盘控制）
        self.command_publishers = {}
        for drone_id in drone_ids:
            topic = f'/drone_{drone_id}/command'
            self.command_publishers[drone_id] = self.create_publisher(
                String,
                topic,
                10
            )
        
        # ✅ 发布扫描控制（Bool 類型，與 fast_scan_node.py 配合）
        self.scan_control_publishers = {}
        for drone_id in drone_ids:
            topic = f'/drone_{drone_id}/scan_control'
            self.scan_control_publishers[drone_id] = self.create_publisher(
                Bool,
                topic,
                10
            )
            self.get_logger().info(f"扫描控制发布到 {topic}")
            self.get_logger().info(f"命令发布到 {topic}")
        
        # 为每个无人机创建独立的决策线程
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
        
        # 为每个无人机创建独立的速度发布线程（100Hz）
        self.publish_threads = {}
        for drone_id in drone_ids:
            thread = threading.Thread(
                target=self.velocity_publish_loop,
                args=(drone_id,),
                daemon=True,
                name=f"PublishThread-Drone{drone_id}"
            )
            thread.start()
            self.publish_threads[drone_id] = thread
        
        # ✅ 新增：訊號歷史記錄器（每次飛行一個 CSV）
        self._init_signal_logger(drone_ids)
        
        # 初始化键盘控制
        self.init_keyboard_control(drone_ids)
        
        self.get_logger().info(f"多无人机信号优化器启动完成（{num_drones} 架无人机）")
        self.get_logger().info("键盘控制已启用，按 ? 查看帮助")
    
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
        """接收 link_quality 数据（每个无人机独立回调）"""
        try:
            data = json.loads(msg.data)
            state = self.drone_states[drone_id]
            state.update_tracker_data(data)
            
            # ✅ 記錄訊號數據到 CSV（每次收到都記錄）
            self._log_signal_data(drone_id, data)
            
            target_id = data.get('target_id', 'unknown')
            self.get_logger().info(
                f"Drone {drone_id}: 收到 {target_id} 信号 "
                f"(RSSI: {data.get('forward_rssi', 0):.1f}, "
                f"SNR: {data.get('forward_snr', 0):.1f})"
            )
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Drone {drone_id}: JSON 解析错误: {e}")
        except Exception as e:
            self.get_logger().error(f"Drone {drone_id}: 回调异常: {e}")
    
    def position_callback(self, msg: VehicleLocalPosition, drone_id: int):
        """接收位置信息（NED坐标系）"""
        state = self.drone_states[drone_id]
        with state.lock:
            state.current_position = Position(
                x=msg.x,
                y=msg.y,
                z=msg.z
            )
            
            # 邏輯修正：
            # 1. 如果還沒開始移動 (movement_count == 0)，持續更新起飛位置
            #    這滿足「空中懸停切入」場景：切入前一直在懸停，位置就是原點
            # 2. 如果是地面自動起飛，velocity_control 會先執行起飛
            #    此時 movement_count 還是 0，直到 optimizer 發出第一個指令
            #    所以我們需要一個標誌位來鎖定原點
            
            if state.takeoff_position is None or state.movement_count == 0:
                state.takeoff_position = Position(
                    x=msg.x,
                    y=msg.y,
                    z=msg.z
                )
                # 降低日誌頻率，只在位置變化大時打印
                # self.get_logger().info(...)
    
    def publish_velocity(self, drone_id: int, vx: float, vy: float, vz: float):
        """发布速度指令（NED frame）"""
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = vz
        cmd.angular.z = 0.0  # 不改变航向
        
        self.velocity_publishers[drone_id].publish(cmd)
    
    def decision_loop(self, drone_id: int):
        """
        單個無人機的決策循環（獨立線程）
        
        掃描序列狀態機：
        INIT → ORIGIN_SCAN → MOVE_FRONT → FRONT_SCAN → MOVE_BACK → BACK_SCAN
             → MOVE_LEFT → LEFT_SCAN → MOVE_RIGHT → RIGHT_SCAN
             → MOVE_TO_BEST → TRACKING → DONE
        """
        self.get_logger().info(f"Drone {drone_id}: 決策線程啟動（邊界掃描模式）")
        state = self.drone_states[drone_id]
        
        while self.running:
            try:
                # 檢查優化是否啟用
                if not self.optimization_enabled:
                    with state.lock:
                        state.current_velocity = (0.0, 0.0, 0.0)
                    time.sleep(0.5)
                    continue
                
                # 根據當前階段執行對應邏輯
                phase = state.scan_phase
                
                # ========== INIT: 等待位置數據並鎖定原點 ==========
                if phase == ScanPhase.INIT:
                    with state.lock:
                        if state.current_position.x == 0.0 and state.current_position.y == 0.0:
                            # 尚未收到位置數據
                            time.sleep(0.5)
                            continue
                        # 鎖定原點 XY
                        state.origin_xy = (state.current_position.x, state.current_position.y)
                        state.scan_scores = {}  # 清空評分
                        state.scan_phase = ScanPhase.ORIGIN_SCAN
                    self.get_logger().info(
                        f"Drone {drone_id}: 🎯 原點鎖定 ({state.origin_xy[0]:.2f}, {state.origin_xy[1]:.2f})，開始原點掃描"
                    )
                
                # ========== ORIGIN_SCAN: 原點掃描（等待兩個 Tracker 更新） ==========
                elif phase == ScanPhase.ORIGIN_SCAN:
                    with state.lock:
                        state.current_velocity = (0.0, 0.0, 0.0)  # 懸停
                        
                        # ✅ 初始化掃描計時器（與 _wait_and_scan 一致）
                        if state.scan_start_time == 0.0:
                            state.scan_start_time = time.time()
                            state.reset_tracker_flags()  # 清除舊數據
                            self.get_logger().info(
                                f"Drone {drone_id}: ⏳ 原點掃描：穩定等待 {self.STABILIZE_WAIT}s 後開始"
                            )
                    
                    elapsed = time.time() - state.scan_start_time
                    
                    # ✅ 穩定等待期間不處理訊號
                    if elapsed < self.STABILIZE_WAIT:
                        with state.lock:
                            state.reset_tracker_flags()  # 持續清除舊數據
                        time.sleep(0.1)
                        continue
                    
                    # 穩定等待結束後檢查兩個 Tracker 是否都就緒
                    if state.both_trackers_ready():
                        quality = state.calculate_current_quality()
                        state.scan_scores['origin'] = quality
                        state.reset_tracker_flags()
                        state.scan_start_time = 0.0  # ✅ 重置計時器
                        state.scan_phase = ScanPhase.MOVE_FRONT
                        self.get_logger().info(
                            f"Drone {drone_id}: 📍 原點掃描完成，品質={quality:.3f}，準備移動到前方"
                        )
                    # ✅ 添加超時保護
                    elif elapsed > self.SCAN_WAIT_TIMEOUT:
                        self.get_logger().warn(
                            f"Drone {drone_id}: ⚠️ 原點掃描超時（{elapsed:.1f}s），記 0 分並繼續"
                        )
                        state.scan_scores['origin'] = 0.0
                        state.reset_tracker_flags()
                        state.scan_start_time = 0.0
                        state.scan_phase = ScanPhase.MOVE_FRONT
                    else:
                        time.sleep(0.5)
                        continue
                
                # ========== MOVE_FRONT: 移動到前方邊界 ==========
                elif phase == ScanPhase.MOVE_FRONT:
                    self._move_to_scan_point(drone_id, 'front', ScanPhase.FRONT_SCAN)
                
                # ========== FRONT_SCAN: 前方掃描 ==========
                elif phase == ScanPhase.FRONT_SCAN:
                    self._wait_and_scan(drone_id, 'front', ScanPhase.MOVE_BACK)
                
                # ========== MOVE_BACK: 移動到後方邊界（經過原點） ==========
                elif phase == ScanPhase.MOVE_BACK:
                    self._move_to_scan_point(drone_id, 'back', ScanPhase.BACK_SCAN)
                
                # ========== BACK_SCAN: 後方掃描 ==========
                elif phase == ScanPhase.BACK_SCAN:
                    self._wait_and_scan(drone_id, 'back', ScanPhase.RETURN_TO_ORIGIN, next_target='left')
                
                # ========== RETURN_TO_ORIGIN: 返回原點（中繼點） ==========
                elif phase == ScanPhase.RETURN_TO_ORIGIN:
                    self._move_to_origin(drone_id)
                
                # ========== MOVE_LEFT: 移動到左方邊界 ==========
                elif phase == ScanPhase.MOVE_LEFT:
                    self._move_to_scan_point(drone_id, 'left', ScanPhase.LEFT_SCAN)
                
                # ========== LEFT_SCAN: 左方掃描 ==========
                elif phase == ScanPhase.LEFT_SCAN:
                    self._wait_and_scan(drone_id, 'left', ScanPhase.RETURN_TO_ORIGIN, next_target='right')
                
                # ========== MOVE_RIGHT: 移動到右方邊界 ==========
                elif phase == ScanPhase.MOVE_RIGHT:
                    self._move_to_scan_point(drone_id, 'right', ScanPhase.RIGHT_SCAN)
                
                # ========== RIGHT_SCAN: 右方掃描 ==========
                elif phase == ScanPhase.RIGHT_SCAN:
                    self._wait_and_scan(drone_id, 'right', ScanPhase.CHOOSE_BEST)
                
                # ========== CHOOSE_BEST: 選擇最佳點並設定目標 ==========
                elif phase == ScanPhase.CHOOSE_BEST:
                    self._select_best_point(drone_id)
                
                # ========== MOVE_TO_BEST: 移動到最佳點（持續檢查距離） ==========
                elif phase == ScanPhase.MOVE_TO_BEST:
                    self._move_to_best_point(drone_id)
                
                # ========== HOVERING: 懸停監控（已到達最佳點） ==========
                elif phase == ScanPhase.HOVERING:
                    with state.lock:
                        state.current_velocity = (0.0, 0.0, 0.0)  # 懸停
                    
                    if state.both_trackers_ready():
                        quality = state.calculate_current_quality()
                        best_dir = state.best_direction
                        best_score = state.scan_scores.get(best_dir, 0.0)
                        
                        self.get_logger().info(
                            f"Drone {drone_id}: 📍 懸停中 - 當前品質={quality:.3f}，最佳點={best_dir}({best_score:.3f})"
                        )
                        state.reset_tracker_flags()
                        state.current_quality = quality
                        state.quality_history.append(quality)
                    
                    time.sleep(1.0)
                
                # ========== DONE: 完成 ==========
                elif phase == ScanPhase.DONE:
                    with state.lock:
                        state.current_velocity = (0.0, 0.0, 0.0)
                    time.sleep(1.0)
                
                else:
                    time.sleep(0.5)
                
            except Exception as e:
                self.get_logger().error(f"Drone {drone_id}: 決策循環異常: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(1.0)
        
        self.get_logger().info(f"Drone {drone_id}: 決策線程退出")
    
    def _get_scan_point_offset(self, direction: str) -> Tuple[float, float]:
        """
        獲取掃描點相對於原點的偏移量
        
        Args:
            direction: 'front', 'back', 'left', 'right'
        
        Returns:
            (dx, dy) 相對原點的偏移
        """
        d = self.SCAN_DISTANCE
        offsets = {
            'front': (d, 0.0),    # +X 方向
            'back': (-d, 0.0),    # -X 方向
            'left': (0.0, -d),    # -Y 方向（NED 座標系）
            'right': (0.0, d),    # +Y 方向
            'origin': (0.0, 0.0)
        }
        return offsets.get(direction, (0.0, 0.0))
    
    def _move_to_origin(self, drone_id: int):
        """
        返回原點（單軸移動）
        
        邏輯：
        - 根據 next_scan_target 判斷返回後的下一個目標
        - 到達原點後進入對應的 MOVE_* 階段
        """
        state = self.drone_states[drone_id]
        
        with state.lock:
            if state.origin_xy is None:
                state.scan_phase = ScanPhase.INIT
                return
            
            origin_x, origin_y = state.origin_xy
            current_x = state.current_position.x
            current_y = state.current_position.y
            
            # 計算到原點的距離
            dist = math.sqrt((origin_x - current_x)**2 + (origin_y - current_y)**2)
            
            # ✅ 到達原點判定（GPS 考慮 0.30m 容差）
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
                    state.current_velocity = (0.0, 0.0, 0.0)
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
            if elapsed > self.MOVE_DURATION * 3:
                self.get_logger().warning(
                    f"Drone {drone_id}: ⚠️ 返回原點超時（{elapsed:.1f}s），強制進入下一階段"
                )
                state.scan_phase = ScanPhase.CHOOSE_BEST
                state.current_velocity = (0.0, 0.0, 0.0)
                return
            
            # 計算速度（朝向原點）
            if dist > 0.01:
                vx = (origin_x - current_x) / dist * self.SCAN_VELOCITY
                vy = (origin_y - current_y) / dist * self.SCAN_VELOCITY
            else:
                vx, vy = 0.0, 0.0
            
            # 固定高度
            state.current_velocity = (vx, vy, 0.0)
        
        time.sleep(0.1)
    
    def _move_to_scan_point(self, drone_id: int, direction: str, next_phase: str):
        """
        移動到指定掃描點
        
        Args:
            drone_id: 無人機 ID
            direction: 目標方向 ('front', 'back', 'left', 'right')
            next_phase: 移動完成後的下一階段
        """
        state = self.drone_states[drone_id]
        
        with state.lock:
            if state.origin_xy is None:
                state.scan_phase = ScanPhase.INIT
                return
            
            origin_x, origin_y = state.origin_xy
            current_x = state.current_position.x
            current_y = state.current_position.y
            
            # 目標位置
            dx, dy = self._get_scan_point_offset(direction)
            target_x = origin_x + dx
            target_y = origin_y + dy
            
            # 確保不超出邊界
            target_x = max(origin_x + self.BOUNDS_X[0], min(origin_x + self.BOUNDS_X[1], target_x))
            target_y = max(origin_y + self.BOUNDS_Y[0], min(origin_y + self.BOUNDS_Y[1], target_y))
            
            # 計算距離
            dist_x = target_x - current_x
            dist_y = target_y - current_y
            dist = math.sqrt(dist_x**2 + dist_y**2)
            
            # ✅ 改進的到達判定：距離穩定標準
            if dist < self.ARRIVAL_THRESHOLD:
                if abs(dist - state.last_distance) < 0.05:
                    state.distance_stable_count += 1
                else:
                    state.distance_stable_count = 0
                
                if state.distance_stable_count >= 3:
                    state.current_velocity = (0.0, 0.0, 0.0)
                    state.is_moving = False
                    state.scan_phase = next_phase
                    state.movement_count += 1
                    state.distance_stable_count = 0
                    self.get_logger().info(
                        f"Drone {drone_id}: ✅ 到達 {direction} 掃描點（距離 {dist:.2f}m，穩定）"
                    )
                    return
            elif dist < self.ARRIVAL_THRESHOLD + 0.2:
                if abs(dist - state.last_distance) < 0.05:
                    state.distance_stable_count += 1
                else:
                    state.distance_stable_count = 0
                
                if state.distance_stable_count >= 10:
                    state.current_velocity = (0.0, 0.0, 0.0)
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
            
            # 開始移動或持續移動
            if not state.is_moving:
                state.is_moving = True
                state.move_start_time = time.time()
                self.get_logger().info(
                    f"Drone {drone_id}: 🚀 移動到 {direction} (距離: {dist:.2f}m)"
                )
            
            # 檢查移動超時（防止卡住）
            elapsed = time.time() - state.move_start_time
            if elapsed > self.MOVE_DURATION * 3:
                state.current_velocity = (0.0, 0.0, 0.0)
                state.is_moving = False
                state.scan_phase = next_phase
                state.movement_count += 1
                state.distance_stable_count = 0
                self.get_logger().warning(
                    f"Drone {drone_id}: ⚠️ 移動超時（{direction}，距離 {dist:.2f}m，耗時 {elapsed:.1f}s），強制進入 {direction} 掃描"
                )
                return
            
            # 計算速度（朝向目標）
            if dist > 0.01:
                vx = (dist_x / dist) * self.SCAN_VELOCITY
                vy = (dist_y / dist) * self.SCAN_VELOCITY
            else:
                vx, vy = 0.0, 0.0
            
            # 固定高度（vz = 0）
            state.current_velocity = (vx, vy, 0.0)
        
        time.sleep(0.1)  # 100ms 更新週期
    
    def _wait_and_scan(self, drone_id: int, point_name: str, next_phase: str, next_target: Optional[str] = None):
        """
        等待掃描完成並記錄品質（帶超時保護）
        
        Args:
            drone_id: 無人機 ID
            point_name: 掃描點名稱
            next_phase: 掃描完成後的下一階段
            next_target: ✅ 如果 next_phase 是 RETURN_TO_ORIGIN，此參數指定返回後的目標 ('left' 或 'right')
        """
        state = self.drone_states[drone_id]
        
        with state.lock:
            state.current_velocity = (0.0, 0.0, 0.0)  # 懸停
            
            # 初始化掃描計時器
            if state.scan_start_time == 0.0:
                state.scan_start_time = time.time()
                # ✅ 到達後先等待穩定，清除舊的 Tracker 資料
                state.reset_tracker_flags()
                self.get_logger().info(
                    f"Drone {drone_id}: ⏳ {point_name} 穩定等待 {self.STABILIZE_WAIT}s 後開始掃描..."
                )
        
        # ✅ 穩定等待期間不處理訊號
        elapsed = time.time() - state.scan_start_time
        if elapsed < self.STABILIZE_WAIT:
            time.sleep(0.1)
            return
        
        # 穩定等待結束後，檢查是否兩個 Tracker 都就緒
        if state.both_trackers_ready():
            quality = state.calculate_current_quality()
            state.scan_scores[point_name] = quality
            state.reset_tracker_flags()
            state.scan_start_time = 0.0  # 重置計時器
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
        # 超時保護
        elif elapsed > self.SCAN_WAIT_TIMEOUT:
            self.get_logger().warn(
                f"Drone {drone_id}: ⚠️ {point_name} 掃描超時（{self.SCAN_WAIT_TIMEOUT:.0f}s），記 0 分並繼續"
            )
            state.scan_scores[point_name] = 0.0  # 超時給 0 分
            state.reset_tracker_flags()
            state.scan_start_time = 0.0
            state.scan_phase = next_phase
            
            # ✅ 超時時也要設定 next_scan_target
            if next_phase == ScanPhase.RETURN_TO_ORIGIN and next_target:
                state.next_scan_target = next_target
        else:
            time.sleep(0.5)  # 繼續等待
    
    def _select_best_point(self, drone_id: int):
        """
        選擇最佳掃描點並設定目標位置
        
        Args:
            drone_id: 無人機 ID
        """
        state = self.drone_states[drone_id]
        
        with state.lock:
            scores = state.scan_scores
            if not scores:
                state.scan_phase = ScanPhase.DONE
                self.get_logger().warn(f"Drone {drone_id}: 無掃描數據，結束")
                return
            
            # 找出最佳方向
            best_dir = max(scores, key=scores.get)
            best_score = scores[best_dir]
            state.best_direction = best_dir
            
            self.get_logger().info(
                f"Drone {drone_id}: 🏆 五點掃描結果: {scores}"
            )
            self.get_logger().info(
                f"Drone {drone_id}: 🎯 最佳方向: {best_dir} (品質={best_score:.3f})"
            )
            
            # 如果最佳是原點，直接進入懸停
            if best_dir == 'origin':
                state.target_xy = state.origin_xy
                state.scan_phase = ScanPhase.HOVERING
                self.get_logger().info(
                    f"Drone {drone_id}: 📍 原點最佳，保持原地懸停"
                )
                return
            
            # 計算目標位置（相對原點偏移，嚴格限制在 ±1.5m）
            origin_x, origin_y = state.origin_xy
            dx, dy = self._get_scan_point_offset(best_dir)
            
            # 目標位置 = 原點 + 偏移，但不超出邊界
            target_x = origin_x + dx
            target_y = origin_y + dy
            
            # 嚴格邊界檢查（±1.5m）
            target_x = max(origin_x + self.BOUNDS_X[0], min(origin_x + self.BOUNDS_X[1], target_x))
            target_y = max(origin_y + self.BOUNDS_Y[0], min(origin_y + self.BOUNDS_Y[1], target_y))
            
            state.target_xy = (target_x, target_y)
            state.is_moving = False
            state.scan_phase = ScanPhase.MOVE_TO_BEST
            
            self.get_logger().info(
                f"Drone {drone_id}: 🚀 準備移動到最佳點 ({target_x:.2f}, {target_y:.2f})"
            )
    
    def _move_to_best_point(self, drone_id: int):
        """
        移動到最佳點（持續檢查距離，到達後停止）
        
        Args:
            drone_id: 無人機 ID
        """
        state = self.drone_states[drone_id]
        
        with state.lock:
            if state.target_xy is None:
                state.scan_phase = ScanPhase.HOVERING
                return
            
            target_x, target_y = state.target_xy
            current_x = state.current_position.x
            current_y = state.current_position.y
            
            # 計算與目標的距離
            dist_x = target_x - current_x
            dist_y = target_y - current_y
            dist = math.sqrt(dist_x**2 + dist_y**2)
            
            # 如果已到達目標（誤差 < ARRIVAL_THRESHOLD）
            if dist < self.ARRIVAL_THRESHOLD:
                state.current_velocity = (0.0, 0.0, 0.0)
                state.is_moving = False
                state.scan_phase = ScanPhase.HOVERING
                self.get_logger().info(
                    f"Drone {drone_id}: ✅ 已到達最佳點 {state.best_direction}，開始懸停監控"
                )
                self.get_logger().info(
                    f"Drone {drone_id}: 🎯 五點掃描完成！最佳信號位置: {state.best_direction} "
                    f"(品質={state.scan_scores.get(state.best_direction, 0):.3f})"
                )
                self.get_logger().info(
                    f"Drone {drone_id}: ⌨️  按 L 降落 | 按 F 重新掃描 | 按 H 緊急停止"
                )
                return
            
            # 開始移動或持續移動
            if not state.is_moving:
                state.is_moving = True
                state.move_start_time = time.time()
                self.get_logger().info(
                    f"Drone {drone_id}: 🚁 移動中... (距離: {dist:.2f}m)"
                )
            
            # 檢查移動超時（防止卡住）
            if time.time() - state.move_start_time > self.MOVE_DURATION * 5:
                state.current_velocity = (0.0, 0.0, 0.0)
                state.is_moving = False
                state.scan_phase = ScanPhase.HOVERING
                self.get_logger().warn(
                    f"Drone {drone_id}: ⚠️ 移動超時，強制進入懸停模式"
                )
                return
            
            # 計算速度（朝向目標）
            if dist > 0.05:
                vx = (dist_x / dist) * self.SCAN_VELOCITY
                vy = (dist_y / dist) * self.SCAN_VELOCITY
            else:
                vx, vy = 0.0, 0.0
            
            # 嚴格邊界檢查：預測下一位置是否超出 ±1.5m
            origin_x, origin_y = state.origin_xy
            next_x = current_x + vx * 0.5  # 預測 0.5 秒後
            next_y = current_y + vy * 0.5
            
            # 檢查 X 邊界
            if next_x < origin_x + self.BOUNDS_X[0] or next_x > origin_x + self.BOUNDS_X[1]:
                vx = 0.0
                self.get_logger().warn(f"Drone {drone_id}: X 軸到達邊界，停止 X 方向移動")
            
            # 檢查 Y 邊界
            if next_y < origin_y + self.BOUNDS_Y[0] or next_y > origin_y + self.BOUNDS_Y[1]:
                vy = 0.0
                self.get_logger().warn(f"Drone {drone_id}: Y 軸到達邊界，停止 Y 方向移動")
            
            # 如果兩軸都被清零，直接進入懸停
            if vx == 0.0 and vy == 0.0:
                state.is_moving = False
                state.scan_phase = ScanPhase.HOVERING
                self.get_logger().info(
                    f"Drone {drone_id}: 📍 到達邊界，進入懸停模式"
                )
                return
            
            # 固定高度（vz = 0）
            state.current_velocity = (vx, vy, 0.0)
        
        time.sleep(0.1)  # 100ms 更新週期
    
    def velocity_publish_loop(self, drone_id: int):
        """
        持续发布速度指令（100Hz）
        
        PX4 Offboard 模式要求持续发布，否则会触发 failsafe
        """
        rate = 1.0 / self.PUBLISH_RATE
        state = self.drone_states[drone_id]
        
        while self.running:
            try:
                with state.lock:
                    vx, vy, vz = state.current_velocity
                
                self.publish_velocity(drone_id, vx, vy, vz)
                time.sleep(rate)
                
            except Exception as e:
                self.get_logger().error(f"Drone {drone_id}: 发布循环异常: {e}")
                time.sleep(rate)
    
    def destroy_node(self):
        """节点关闭"""
        self.get_logger().info("正在关闭多无人机优化器...")
        self.running = False
        self.keyboard_running = False
        
        # ✅ 關閉訊號記錄檔案
        self._close_signal_loggers()
        
        # 停止所有线程
        for drone_id, thread in self.decision_threads.items():
            thread.join(timeout=2.0)
        for drone_id, thread in self.publish_threads.items():
            thread.join(timeout=2.0)
        if self.keyboard_thread:
            self.keyboard_thread.join(timeout=2.0)
        
        # 恢复终端设置
        if sys.platform != 'win32' and self.terminal_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.terminal_settings)
        
        Node.destroy_node(self)


def main(args=None):
    rclpy.init(args=args)
    
    node = MultiDroneSignalOptimizer()
    
    # 使用多线程执行器
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        # ✅ 自定義 spin 邏輯，監控 running 標誌
        while rclpy.ok() and node.running:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("收到中断信号")
    finally:
        node.destroy_node()
        # ✅ 檢查 ROS 是否已初始化再進行 shutdown
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
