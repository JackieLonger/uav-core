#!/usr/bin/env python3
"""
Signal Optimizer Node
- 訂閱 fast_scan_node 發布的訊號品質 (JSON String)
- 根據 SNR/RSSI 優化無人機位置
- 發布位置指令至 PX4
"""

import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus
from std_msgs.msg import String
import numpy as np
import json


class SignalOptimizerNode(Node):
    def __init__(self):
        super().__init__('signal_optimizer_node')
        
        # 訂閱 Meshtastic 訊號品質 (fast_scan_node 發布的 JSON String)
        self.signal_sub = self.create_subscription(
            String,
            '/link_quality',
            self.signal_callback,
            10
        )
        
        # PX4 通訊
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            10
        )
        
        # 優化參數
        self.search_step = 0.3  # meters
        self.best_signal = {
            'snr': float('-inf'),
            'rssi': float('-inf'),
            'position': None
        }
        self.current_position = {
            'x': 0.0,
            'y': 0.0,
            'z': 2.0
        }
        
        # 搜索範圍
        self.search_bounds = {
            'x_min': -1.5, 'x_max': 1.5,
            'y_min': -1.5, 'y_max': 1.5,
            'z': 2.0
        }
        
        # Vehicle status
        self.vehicle_status = None
        
        # 開始定期優化
        self.optimization_timer = self.create_timer(5.0, self.optimize_position)
        self.offboard_control_timer = self.create_timer(0.1, self.publish_offboard_mode)
        
        self.get_logger().info('Signal optimizer node is ready')
    
    def vehicle_status_callback(self, msg):
        """處理飛行器狀態"""
        self.vehicle_status = msg
        
    def signal_callback(self, msg):
        """
        處理訊號品質數據 (JSON String from fast_scan_node)
        Expected JSON format:
        {
            "timestamp": "2025-10-20 14:30:45",
            "target_id": "!e2e5b980",
            "status": "Success",
            "forward_rssi": -96.3,
            "forward_snr": 7.2,
            "return_rssi": -95,
            "return_snr": 8.5
        }
        """
        try:
            data = json.loads(msg.data)
            
            # 檢查狀態
            if data.get('status') != 'Success':
                return
            
            # 使用 return SNR 作為主要指標
            current_snr = data.get('return_snr', float('-inf'))
            current_rssi = data.get('return_rssi', float('-inf'))
            
            # 組合評分: SNR 更重要 (權重 0.7), RSSI 權重 0.3
            signal_score = current_snr * 0.7 + (current_rssi / -50.0) * 100 * 0.3
            
            # 更新最佳訊號記錄
            if signal_score > self.best_signal.get('score', float('-inf')):
                self.best_signal['snr'] = current_snr
                self.best_signal['rssi'] = current_rssi
                self.best_signal['score'] = signal_score
                self.best_signal['position'] = self.current_position.copy()
                self.get_logger().info(
                    f'New best signal: SNR={current_snr:.1f}, '
                    f'Est.RSSI={current_rssi:.1f}, Score={signal_score:.1f}'
                )
        except json.JSONDecodeError:
            self.get_logger().warning(f'Invalid JSON in signal data: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error processing signal data: {e}')
    
    def optimize_position(self):
        """執行位置優化"""
        if not self.vehicle_status or self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return
            
        new_position = self.get_next_position()
        if new_position:
            self.publish_trajectory_setpoint(
                new_position['x'],
                new_position['y'],
                new_position['z']
            )
            self.current_position = new_position
            self.get_logger().info(f'Moving to position: {new_position}')
    
    def get_next_position(self):
        """生成下一個搜索位置"""
        while True:
            dx = np.random.uniform(-self.search_step, self.search_step)
            dy = np.random.uniform(-self.search_step, self.search_step)
            
            new_x = self.current_position['x'] + dx
            new_y = self.current_position['y'] + dy
            
            if (self.search_bounds['x_min'] <= new_x <= self.search_bounds['x_max'] and
                self.search_bounds['y_min'] <= new_y <= self.search_bounds['y_max']):
                return {
                    'x': new_x,
                    'y': new_y,
                    'z': self.search_bounds['z']
                }
    
    def publish_offboard_mode(self):
        """發布 Offboard 控制模式"""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.publisher_offboard_mode.publish(msg)
    
    def publish_trajectory_setpoint(self, x, y, z):
        """發布位置指令"""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0  # 保持當前航向
        self.publisher_trajectory.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SignalOptimizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()