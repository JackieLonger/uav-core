#!/usr/bin/env python3
"""
Multi-Drone Signal Optimizer - 多无人机协同信号优化器

架构：
- 笔记本运行此节点（集中决策）
- 每个 Jetson 运行 fast_scan_node.py（扫描并发布到 /drone_N/link_quality）
- 每个 Jetson 运行 velocity_control.py（接收 /drone_N/offboard_velocity_cmd）
- 多线程异步处理每个无人机的决策（不阻塞）

策略：
1. 优先上升到 1.5m
2. RSSI + SNR 综合评分（各占50%）
3. 第n次 > 第n-1次 → 继续方向
4. 信号变差 → XY随机小幅搜索
5. 最多 5 次移动
6. 速度限制 0.3 m/s
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition
import threading
import json
import time
import math
import random
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Dict, Tuple


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
        """综合质量评分（RSSI + SNR 各占 50%）"""
        # RSSI 归一化 (-100 to -20 dBm)
        rssi_avg = (self.forward_rssi + self.return_rssi) / 2.0
        rssi_score = (rssi_avg + 100) / 80.0
        rssi_score = max(0.0, min(1.0, rssi_score))
        
        # SNR 归一化 (-10 to 20 dB)
        snr_avg = (self.forward_snr + self.return_snr) / 2.0
        snr_score = (snr_avg + 10) / 30.0
        snr_score = max(0.0, min(1.0, snr_score))
        
        # 综合评分
        return 0.5 * rssi_score + 0.5 * snr_score


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
            # 跳过失败的扫描
            if json_data.get('status') != 'Success':
                return

            target_id = json_data['target_id']
            
            # 若尚未收集满两个 tracker，加入新 ID
            if target_id not in self.dynamic_tracker_ids and len(self.dynamic_tracker_ids) < 2:
                self.dynamic_tracker_ids.append(target_id)
            
            # 若收到不在本无人机绑定范围的第三种 ID，直接忽略
            if target_id not in self.dynamic_tracker_ids:
                return
            
            tracker_data = TrackerData(
                target_id=target_id,
                forward_rssi=json_data.get('forward_rssi', 0.0),
                forward_snr=json_data.get('forward_snr', 0.0),
                return_rssi=json_data.get('return_rssi', 0.0),
                return_snr=json_data.get('return_snr', 0.0),
                timestamp=json_data.get('timestamp', ''),
                status=json_data.get('status', 'Unknown')
            )
            
            self.tracker_data_map[target_id] = tracker_data
            self.tracker_ready_map[target_id] = True
    
    def calculate_current_quality(self) -> float:
        """计算当前综合质量"""
        with self.lock:
            if len(self.dynamic_tracker_ids) < 2:
                return 0.0
            
            a_id, b_id = self.dynamic_tracker_ids
            if a_id not in self.tracker_data_map or b_id not in self.tracker_data_map:
                return 0.0
            
            quality_a = self.tracker_data_map[a_id].quality_score
            quality_b = self.tracker_data_map[b_id].quality_score
            
            # 平均质量
            return (quality_a + quality_b) / 2.0


class MultiDroneSignalOptimizer(Node):
    """多无人机信号优化器（多线程异步）"""
    
    # 常量配置
    MAX_MOVEMENTS = 5
    MAX_VELOCITY = 0.3  # m/s
    ALTITUDE_THRESHOLD = 0.2  # 初始上升閾值 (0.2m)，避免強制爬升太高
    PUBLISH_RATE = 100.0  # Hz (100Hz for PX4 Offboard)
    
    # 边界（相对于 takeoff_position）
    BOUNDS_X = (-1.5, 1.5)
    BOUNDS_Y = (-1.5, 1.5)
    BOUNDS_Z = (0.0, 1.5)  # 允許在原點上方 0~1.5m 範圍內搜索
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
        super().__init__('multi_drone_signal_optimizer')
        
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
        
        self.get_logger().info(f"多无人机信号优化器启动完成（{num_drones} 架无人机）")
    
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
        
        # 等待所有线程退出
        for drone_id, thread in self.decision_threads.items():
            thread.join(timeout=2.0)
        for drone_id, thread in self.publish_threads.items():
            thread.join(timeout=2.0)
        
        super().destroy_node()


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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
