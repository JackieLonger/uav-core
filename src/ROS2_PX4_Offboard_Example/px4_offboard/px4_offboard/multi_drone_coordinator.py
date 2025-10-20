#!/usr/bin/env python3
"""
Multi-Drone Coordinator
功能:
1. 訂閱所有無人機的優化器狀態
2. 分析無人機位置和搜索進度
3. 發布協調指令 (避衝、區域分配)
4. 監測訊號融合狀態

使用:
ros2 run px4_offboard multi_drone_coordinator.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from typing import Dict, List, Tuple


class MultiDroneCoordinator(Node):
    def __init__(self):
        super().__init__('multi_drone_coordinator')
        
        # 參數
        self.declare_parameter('max_drones', 4)
        self.declare_parameter('min_separation', 0.5)  # 最小安全距離 (m)
        self.declare_parameter('enable_coordination', True)
        
        self.max_drones = self.get_parameter('max_drones').value
        self.min_separation = self.get_parameter('min_separation').value
        self.enable_coordination = self.get_parameter('enable_coordination').value
        
        # 無人機狀態字典
        self.drones: Dict[str, Dict] = {}
        
        # 發布協調策略
        self.strategy_pub = self.create_publisher(
            String,
            '/multi_drone/strategy',
            10
        )
        
        # 定時檢查
        self.create_timer(2.0, self.coordinate_drones)
        self.create_timer(5.0, self.publish_status)
        
        self.get_logger().info(
            f'🤖 Multi-Drone Coordinator Ready | '
            f'Max Drones: {self.max_drones}, '
            f'Min Separation: {self.min_separation}m'
        )
    
    def subscribe_to_drone(self, drone_id: str):
        """動態訂閱新無人機"""
        if drone_id not in self.drones:
            self.create_subscription(
                String,
                f'/{drone_id}/optimizer_status',
                lambda msg, did=drone_id: self.drone_status_callback(msg, did),
                10
            )
            self.drones[drone_id] = {
                'position': None,
                'best_signal': None,
                'mode': 'unknown',
                'last_update': time.time()
            }
            self.get_logger().info(f'📡 訂閱無人機: {drone_id}')
    
    def drone_status_callback(self, msg: String, drone_id: str):
        """接收無人機狀態更新"""
        try:
            data = json.loads(msg.data)
            
            # 自動發現新無人機
            if drone_id not in self.drones:
                self.subscribe_to_drone(drone_id)
            
            # 更新狀態
            self.drones[drone_id].update({
                'position': data.get('current_position'),
                'best_signal': data.get('best_signal'),
                'mode': data.get('mode'),
                'search_count': data.get('search_count'),
                'elapsed_time': data.get('elapsed_time'),
                'last_update': time.time()
            })
            
        except Exception as e:
            self.get_logger().debug(f'Drone status callback error: {e}')
    
    def coordinate_drones(self):
        """執行多機協調"""
        if not self.enable_coordination or len(self.drones) < 2:
            return
        
        # 檢查無人機距離
        collisions = self.detect_collisions()
        if collisions:
            self.handle_collisions(collisions)
        
        # 優化搜索區域分配
        allocation = self.allocate_search_regions()
        if allocation:
            self.publish_allocation(allocation)
    
    def detect_collisions(self) -> List[Tuple[str, str]]:
        """檢測無人機間距離過近的情況"""
        collisions = []
        drone_ids = list(self.drones.keys())
        
        for i in range(len(drone_ids)):
            for j in range(i + 1, len(drone_ids)):
                d1, d2 = drone_ids[i], drone_ids[j]
                
                pos1 = self.drones[d1].get('position')
                pos2 = self.drones[d2].get('position')
                
                if not pos1 or not pos2:
                    continue
                
                distance = self.calculate_distance(pos1, pos2)
                
                if distance < self.min_separation:
                    collisions.append((d1, d2, distance))
                    self.get_logger().warn(
                        f'⚠️ 衝突警告: {d1} 和 {d2} 距離 = {distance:.2f}m '
                        f'(閾值: {self.min_separation}m)'
                    )
        
        return collisions
    
    def handle_collisions(self, collisions: List[Tuple[str, str, float]]):
        """處理衝突 - 調整無人機搜索方向"""
        for d1, d2, distance in collisions:
            # 生成避衝指令
            pos1 = self.drones[d1]['position']
            pos2 = self.drones[d2]['position']
            
            # 計算避衝方向 (相互遠離)
            escape_dir1 = self.calculate_escape_direction(pos2, pos1)
            escape_dir2 = self.calculate_escape_direction(pos1, pos2)
            
            strategy = {
                'action': 'avoid',
                'd1_escape': escape_dir1,
                'd2_escape': escape_dir2,
                'target_separation': self.min_separation * 1.5
            }
            
            msg = String()
            msg.data = json.dumps(strategy, ensure_ascii=False)
            self.strategy_pub.publish(msg)
    
    def allocate_search_regions(self) -> Dict[str, Dict]:
        """分配不重疊的搜索區域"""
        if len(self.drones) < 2:
            return {}
        
        allocation = {}
        num_drones = len(self.drones)
        drone_ids = sorted(self.drones.keys())
        
        # 將 2×2 區域分割成 N 個象限
        if num_drones == 2:
            allocation[drone_ids[0]] = {'x_min': -1.0, 'x_max': 0.0, 'y_min': -1.0, 'y_max': 1.0}
            allocation[drone_ids[1]] = {'x_min': 0.0, 'x_max': 1.0, 'y_min': -1.0, 'y_max': 1.0}
        elif num_drones == 3:
            # Y 軸分層
            allocation[drone_ids[0]] = {'x_min': -1.0, 'x_max': 1.0, 'y_min': -1.0, 'y_max': -0.33}
            allocation[drone_ids[1]] = {'x_min': -1.0, 'x_max': 1.0, 'y_min': -0.33, 'y_max': 0.33}
            allocation[drone_ids[2]] = {'x_min': -1.0, 'x_max': 1.0, 'y_min': 0.33, 'y_max': 1.0}
        elif num_drones == 4:
            # 2×2 四象限
            allocation[drone_ids[0]] = {'x_min': -1.0, 'x_max': 0.0, 'y_min': -1.0, 'y_max': 0.0}
            allocation[drone_ids[1]] = {'x_min': 0.0, 'x_max': 1.0, 'y_min': -1.0, 'y_max': 0.0}
            allocation[drone_ids[2]] = {'x_min': -1.0, 'x_max': 0.0, 'y_min': 0.0, 'y_max': 1.0}
            allocation[drone_ids[3]] = {'x_min': 0.0, 'x_max': 1.0, 'y_min': 0.0, 'y_max': 1.0}
        
        return allocation
    
    def publish_allocation(self, allocation: Dict[str, Dict]):
        """發布區域分配"""
        for drone_id, region in allocation.items():
            strategy = {
                'action': 'allocate_region',
                'region': region,
                'priority': 'medium'
            }
            
            # 發布到協調 topic
            msg = String()
            msg.data = json.dumps(strategy, ensure_ascii=False)
            self.strategy_pub.publish(msg)
    
    def publish_status(self):
        """定期發布協調器狀態"""
        # 移除超時的無人機 (> 5 秒未更新)
        current_time = time.time()
        timeout_drones = [
            did for did, info in self.drones.items()
            if current_time - info['last_update'] > 5.0
        ]
        
        for did in timeout_drones:
            self.get_logger().warn(f'⏱️ 無人機 {did} 超時 (未見 5 秒)')
            del self.drones[did]
        
        # 統計
        num_drones = len(self.drones)
        active_count = sum(1 for d in self.drones.values() if d['mode'] == 'search')
        hold_count = sum(1 for d in self.drones.values() if d['mode'] == 'hold')
        
        if num_drones > 0:
            self.get_logger().info(
                f'📊 協調狀態: {num_drones} 架無人機 | '
                f'搜索中: {active_count} | 停留中: {hold_count}'
            )
    
    @staticmethod
    def calculate_distance(pos1: Dict, pos2: Dict) -> float:
        """計算 3D 距離"""
        import math
        dx = pos1['x'] - pos2['x']
        dy = pos1['y'] - pos2['y']
        dz = pos1['z'] - pos2['z']
        return math.sqrt(dx**2 + dy**2 + dz**2)
    
    @staticmethod
    def calculate_escape_direction(from_pos: Dict, to_pos: Dict) -> Dict:
        """計算逃逸方向 (從 from_pos 遠離 to_pos)"""
        import math
        dx = to_pos['x'] - from_pos['x']
        dy = to_pos['y'] - from_pos['y']
        
        # 反向
        dx = -dx
        dy = -dy
        
        # 正規化
        distance = math.sqrt(dx**2 + dy**2)
        if distance > 0:
            dx /= distance
            dy /= distance
        
        return {'dx': round(dx, 2), 'dy': round(dy, 2)}


def main(args=None):
    rclpy.init(args=args)
    node = MultiDroneCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
