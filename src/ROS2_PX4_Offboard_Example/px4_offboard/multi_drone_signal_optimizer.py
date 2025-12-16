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
            
            # 安全地轉換 RSSI/SNR 為浮點數（可能是字符串或空值）
            def safe_float(val):
                try:
                    return float(val) if val else 0.0
                except (ValueError, TypeError):
                    return 0.0
            
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
基本控制（自動化流程）：
  SPACE  : 解锁 + 起飛 2.5m + 進入 Offboard 模式
  L      : 降落（降落後自動上鎖）
  H      : 紧急悬停（立即停止移动）

扫描控制：
  F      : 启动信号扫描（Fast Scan）
  G      : 停止信号扫描

优化控制：
  S      : 开始信号优化（默认已启用）
  P      : 暂停信号优化

选择控制：
  1/2/3  : 选择单个无人机（Drone 1/2/3）
  A      : 选择所有无人机

其他：
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
        
        # 状态显示定时器（每 2 秒刷新）
        self.status_timer = None
        
    def init_keyboard_control(self, drone_ids: list):
        """初始化键盘控制（在 ROS2 node 初始化后调用）"""
        # 默认选择所有无人机
        self.selected_drones = set(drone_ids)
        
        # 保存终端设置
        if sys.platform != 'win32':
            self.terminal_settings = termios.tcgetattr(sys.stdin)
        
        # 启动键盘监听线程
        self.keyboard_thread = threading.Thread(
            target=self._keyboard_listener,
            daemon=True,
            name="KeyboardListener"
        )
        self.keyboard_thread.start()
        
        # 启动状态显示定时器（5秒刷新一次，降低更新频率）
        self.status_timer = self.create_timer(5.0, self._display_status)
        
        # 显示帮助信息
        print(self.HELP_MSG)
        self._display_status()
        
    def _get_key_nonblocking(self):
        """非阻塞式读取单个按键（Linux/Unix）"""
        if sys.platform == 'win32':
            # Windows 暂不支持
            return None
        
        tty.setraw(sys.stdin.fileno())
        dr, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = None
        if dr:
            key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.terminal_settings)
        return key
    
    def _keyboard_listener(self):
        """键盘监听线程"""
        self.get_logger().info("键盘监听线程启动")
        
        while self.keyboard_running and rclpy.ok():
            try:
                key = self._get_key_nonblocking()
                if key:
                    self._handle_key(key)
            except Exception as e:
                if self.keyboard_running:  # 只在正常运行时报错
                    self.get_logger().error(f"键盘监听异常: {e}")
            time.sleep(0.05)
        
        # 恢复终端设置（安全处理）
        if sys.platform != 'win32' and self.terminal_settings:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.terminal_settings)
            except Exception:
                pass  # 忽略终端恢复错误
        
        self.get_logger().info("键盘监听线程退出")
    
    def _handle_key(self, key: str):
        """处理按键事件"""
        if not self.selected_drones:
            print("⚠️  请先选择无人机（按 1/2/3/A）")
            return
        
        # SPACE: 自動 ARM + TAKEOFF + OFFBOARD
        if key == ' ':
            self._send_command_to_selected('ARM_TOGGLE')
            print(f"✈️  解鎖+起飛+Offboard: {sorted(self.selected_drones)}")
        
        # 降落（降落後自動上鎖）
        elif key == 'l' or key == 'L':
            self._send_command_to_selected('LAND')
            print(f"🛬 降落命令: {sorted(self.selected_drones)}")
        
        # 紧急悬停
        elif key == 'h' or key == 'H':
            self._send_command_to_selected('HOLD')
            print(f"⏸️  紧急悬停: {sorted(self.selected_drones)}")
        
        # 启动扫描
        elif key == 'f' or key == 'F':
            self.scan_enabled = True
            self._send_command_to_selected('START_SCAN')
            print(f"📡 信号扫描已启动: {sorted(self.selected_drones)}")
        
        # 停止扫描
        elif key == 'g' or key == 'G':
            self.scan_enabled = False
            self._send_command_to_selected('STOP_SCAN')
            print(f"⏹️  信号扫描已停止: {sorted(self.selected_drones)}")
        
        # 开始优化
        elif key == 's' or key == 'S':
            self.optimization_enabled = True
            print("🔍 信号优化已启动")
        
        # 暂停优化
        elif key == 'p' or key == 'P':
            self.optimization_enabled = False
            # 停止所有速度指令
            for drone_id in self.selected_drones:
                if drone_id in self.drone_states:
                    with self.drone_states[drone_id].lock:
                        self.drone_states[drone_id].current_velocity = (0.0, 0.0, 0.0)
            print("⏸️  信号优化已暂停")
        
        # 选择单个无人机
        elif key in ['1', '2', '3']:
            drone_id = int(key)
            if drone_id in self.drone_states:
                self.selected_drones = {drone_id}
                print(f"✓ 已选择 Drone {drone_id}")
            else:
                print(f"⚠️  Drone {drone_id} 不存在")
        
        # 选择所有无人机
        elif key == 'a' or key == 'A':
            self.selected_drones = set(self.drone_states.keys())
            print(f"✓ 已选择所有无人机: {sorted(self.selected_drones)}")
        
        # 退出
        elif key == 'q' or key == 'Q':
            print("\n👋 正在退出...")
            self.keyboard_running = False
            self.running = False
            rclpy.shutdown()
        
        # 帮助
        elif key == '?':
            print(self.HELP_MSG)
    
    def _send_command_to_selected(self, command: str):
        """向选中的无人机发送命令"""
        for drone_id in self.selected_drones:
            if drone_id in self.command_publishers:
                msg = String()
                msg.data = command
                self.command_publishers[drone_id].publish(msg)
    
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
    """多无人机信号优化器（多线程异步 + 键盘控制）"""
    
    # 常量配置
    MAX_MOVEMENTS = 5
    MAX_VELOCITY = 0.3  # m/s
    ALTITUDE_THRESHOLD = 1.5  # 优先上升到 1.5m
    PUBLISH_RATE = 100.0  # Hz (100Hz for PX4 Offboard)
    
    # 边界（相对于 takeoff_position）
    BOUNDS_X = (-1.5, 1.5)
    BOUNDS_Y = (-1.5, 1.5)
    BOUNDS_Z = (0.0, 3.0)  # NED: 0.0(起飞点) 到 3.0(起飞点上方3米)
    # 注意：NED 坐标系中，Z 轴向下为正。
    # 如果 takeoff_position.z 是 -10m (海拔 10m)
    # 我們希望飛到 -13m (海拔 13m)
    # 所以相對高度應該是負值。
    # 但這裡的邏輯是基於 current_altitude = takeoff_pos.z - current_pos.z
    # current_altitude > 0 代表在起飛點上方
    # 所以 BOUNDS_Z = (0.0, 3.0) 代表允許在起飛點上方 0~3 米範圍內移動
    BOUNDS_Y = (-1.5, 1.5)
    BOUNDS_Z = (0.0, 3.0)  # 只能上升
    
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
        
        # 初始化键盘控制
        self.init_keyboard_control(drone_ids)
        
        self.get_logger().info(f"多无人机信号优化器启动完成（{num_drones} 架无人机）")
        self.get_logger().info("键盘控制已启用，按 ? 查看帮助")
    
    def link_quality_callback(self, msg: String, drone_id: int):
        """接收 link_quality 数据（每个无人机独立回调）"""
        try:
            data = json.loads(msg.data)
            state = self.drone_states[drone_id]
            state.update_tracker_data(data)
            
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
    
    def calculate_velocity(self, drone_id: int) -> Tuple[float, float, float]:
        """
        计算速度指令（优先上升策略）
        
        策略：
        1. 优先上升到 1.5m
        2. 信号改善 → 继续方向
        3. 信号变差 → XY 随机搜索
        4. 已到高度阈值 → XY 精调
        
        返回: (vx, vy, vz) NED frame (m/s)
        """
        state = self.drone_states[drone_id]
        
        with state.lock:
            # 检查是否达到移动限制
            if state.movement_count >= self.MAX_MOVEMENTS:
                return (0.0, 0.0, 0.0)
            
            # 检查位置数据
            if state.takeoff_position is None:
                return (0.0, 0.0, 0.0)
            
            current_pos = state.current_position
            takeoff_pos = state.takeoff_position
            
            # 计算当前质量
            current_quality = state.calculate_current_quality()
            previous_quality = state.previous_quality
            
            # 判断信号是否改善
            improvement = current_quality > previous_quality
            
            # 计算当前高度（相对于起飞点）
            # NED: z值越小 = 海拔越高
            current_altitude = takeoff_pos.z - current_pos.z
            
            vx, vy, vz = 0.0, 0.0, 0.0
            
            # === 策略 1: 优先上升到 1.5m ===
            # 注意：如果是地面起飛，current_altitude 從 0 開始，會觸發上升
            # 如果是空中懸停切入，takeoff_pos 就是當前高度，current_altitude ≈ 0，也會觸發上升
            # 這符合需求：無論哪種方式，都以「原點」為基準向上搜索
            if current_altitude < self.ALTITUDE_THRESHOLD:
                if improvement or previous_quality == 0.0:
                    # 信号改善或首次移动 → 正常上升
                    vz = -0.3  # NED: 负值 = 上升
                    
                    # 關鍵：一旦決定移動，movement_count 增加，takeoff_position 就會鎖定
                    state.movement_count += 1
                else:
                    # 信号变差但仍需上升 → 慢速上升
                    vz = -0.2
                    state.movement_count += 1
                
                self.get_logger().info(
                    f"Drone {drone_id}: 上升中 ({current_altitude:.2f}m / {self.ALTITUDE_THRESHOLD}m), "
                    f"质量 {current_quality:.3f} → {previous_quality:.3f}"
                )
            
            # === 策略 2: 已到高度阈值或接近顶部 ===
            else:
                if improvement and state.previous_position is not None:
                    # 信号改善 → 继续当前方向
                    direction = current_pos - state.previous_position
                    dist = direction.distance_to(Position(0, 0, 0))
                    
                    if dist > 0.01:
                        direction = direction.normalized()
                        vx = direction.x * 0.3
                        vy = direction.y * 0.3
                        # 可以继续缓慢上升（如果未到顶）
                        if current_altitude < self.BOUNDS_Z[1]:
                            vz = -0.1
                    
                    self.get_logger().info(
                        f"Drone {drone_id}: 信号改善，继续方向 "
                        f"({vx:.2f}, {vy:.2f}, {vz:.2f})"
                    )
                    state.movement_count += 1
                
                else:
                    # 信号变差或无历史 → 随机 XY 搜索
                    vx = random.uniform(-0.2, 0.2)
                    vy = random.uniform(-0.2, 0.2)
                    
                    # 如果还没到顶，可以尝试上升
                    if current_altitude < self.BOUNDS_Z[1] - 0.5:
                        vz = random.choice([0.0, -0.1])
                    
                    self.get_logger().info(
                        f"Drone {drone_id}: 信号变差，随机搜索 "
                        f"({vx:.2f}, {vy:.2f}, {vz:.2f})"
                    )
                    state.movement_count += 1
            
            # === 边界限制 ===
            vx, vy, vz = self.clamp_velocity(
                vx, vy, vz,
                current_pos,
                takeoff_pos
            )
            
            # 更新状态
            state.previous_position = Position(
                current_pos.x,
                current_pos.y,
                current_pos.z
            )
            state.previous_quality = current_quality
            state.current_quality = current_quality
            state.quality_history.append(current_quality)
            
            return (vx, vy, vz)
    
    def clamp_velocity(self, vx: float, vy: float, vz: float,
                      current_pos: Position, takeoff_pos: Position) -> Tuple[float, float, float]:
        """
        限制速度，确保不超出边界
        
        边界检查（绝对坐标）：
        - X: takeoff_pos.x ± 1.5m
        - Y: takeoff_pos.y ± 1.5m
        - Z: takeoff_pos.z to takeoff_pos.z - 3.0m (NED: z值减小 = 上升)
        """
        dt = 0.1  # 预估 0.1s 后的位置
        
        # X 边界检查
        next_x = current_pos.x + vx * dt
        if next_x < takeoff_pos.x + self.BOUNDS_X[0]:
            vx = 0.0
        elif next_x > takeoff_pos.x + self.BOUNDS_X[1]:
            vx = 0.0
        
        # Y 边界检查
        next_y = current_pos.y + vy * dt
        if next_y < takeoff_pos.y + self.BOUNDS_Y[0]:
            vy = 0.0
        elif next_y > takeoff_pos.y + self.BOUNDS_Y[1]:
            vy = 0.0
        
        # Z 边界检查（NED: 向上飞 = z 减小）
        next_z = current_pos.z + vz * dt
        if vz > 0:  # 想要下降
            vz = 0.0  # 不允许下降
        elif next_z < (takeoff_pos.z - self.BOUNDS_Z[1]):  # 超过上限
            vz = 0.0
        
        # 限制总速度大小
        speed = math.sqrt(vx**2 + vy**2 + vz**2)
        if speed > self.MAX_VELOCITY:
            scale = self.MAX_VELOCITY / speed
            vx *= scale
            vy *= scale
            vz *= scale
        
        return (vx, vy, vz)
    
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
        单个无人机的决策循环（独立线程）
        
        每次两个 tracker 信号都到达后才决策
        """
        self.get_logger().info(f"Drone {drone_id}: 决策线程启动")
        state = self.drone_states[drone_id]
        
        while self.running:
            try:
                # 检查优化是否启用
                if not self.optimization_enabled:
                    with state.lock:
                        state.current_velocity = (0.0, 0.0, 0.0)
                    time.sleep(0.5)
                    continue
                
                # 检查是否达到移动限制
                if state.movement_count >= self.MAX_MOVEMENTS:
                    with state.lock:
                        state.current_velocity = (0.0, 0.0, 0.0)
                    time.sleep(1.0)
                    continue
                
                # 等待两个 tracker 信号都到
                if not state.both_trackers_ready():
                    with state.lock:
                        state.current_velocity = (0.0, 0.0, 0.0)
                    time.sleep(0.5)
                    continue
                
                # 计算速度
                vx, vy, vz = self.calculate_velocity(drone_id)
                
                # 更新速度
                with state.lock:
                    state.current_velocity = (vx, vy, vz)
                    state.movement_count += 1
                
                # 重置 tracker flags
                state.reset_tracker_flags()
                
                self.get_logger().info(
                    f"Drone {drone_id}: 移动 {state.movement_count}/{self.MAX_MOVEMENTS}, "
                    f"速度 ({vx:.2f}, {vy:.2f}, {vz:.2f}), "
                    f"质量 {state.current_quality:.3f}"
                )
                
                # 移动约 3 秒后，重置速度为零（悬停等待下一轮扫描）
                time.sleep(3.0)
                
                # 重置速度为零，无人机悬停等待下一轮 Tracker 扫描
                with state.lock:
                    state.current_velocity = (0.0, 0.0, 0.0)
                
                self.get_logger().info(
                    f"Drone {drone_id}: 移动完成，悬停等待下一轮扫描"
                )
                
                # 等待下一轮扫描（约 120 秒）- 已在悬停状态
                time.sleep(1.0)
                
            except Exception as e:
                self.get_logger().error(f"Drone {drone_id}: 决策循环异常: {e}")
                time.sleep(1.0)
        
        self.get_logger().info(f"Drone {drone_id}: 决策线程退出")
    
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
        
        # 停止所有线程
        for drone_id, thread in self.decision_threads.items():
            thread.join(timeout=2.0)
        for drone_id, thread in self.publish_threads.items():
            thread.join(timeout=2.0)
        if self.keyboard_thread:
            self.keyboard_thread.join(timeout=2.0)
        
        # 恢复终端设置（安全处理）
        if sys.platform != 'win32' and self.terminal_settings:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.terminal_settings)
            except Exception:
                pass  # 忽略终端恢复错误
        
        try:
            super().destroy_node()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    
    node = MultiDroneSignalOptimizer()
    
    # 使用多线程执行器
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("收到中断信号")
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # 忽略 shutdown 错误


if __name__ == '__main__':
    main()
