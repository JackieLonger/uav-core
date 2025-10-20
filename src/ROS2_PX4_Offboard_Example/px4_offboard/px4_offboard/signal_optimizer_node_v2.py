#!/usr/bin/env python3
"""
Signal Optimizer Node v2 - 改進版
功能:
1. 快速搜索 + 收斂檢測 (10分鐘內找到最佳點)
2. 停留機制 (找到最佳點後停留)
3. 訊號穩定化 (多次測量平均)
4. 多機準備 (易於擴充)

支援模式:
- search: 主動搜索最佳訊號點
- hold: 停留在最佳點不動
- coordinate: 多機協調模式 (預留)
"""

import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus
from std_msgs.msg import String
import numpy as np
import json
import time
from enum import Enum


class OptimizerMode(Enum):
    """優化器模式"""
    SEARCH = "search"      # 主動搜索
    HOLD = "hold"          # 停留在最佳點
    COORDINATE = "coordinate"  # 多機協調


class SignalOptimizerNodeV2(Node):
    def __init__(self):
        super().__init__('signal_optimizer_node')
        
        # ===== 參數配置 =====
        # 搜索參數 (可動態調整)
        self.declare_parameter('search_mode', 'fast')  # 'fast' or 'thorough'
        self.declare_parameter('convergence_iterations', 5)  # 連續N次無改進則停留
        self.declare_parameter('signal_smooth_window', 3)  # 訊號平滑窗口大小
        self.declare_parameter('drone_id', 'drone_1')  # 無人機編號 (多機用)
        
        self.search_mode = self.get_parameter('search_mode').value
        self.convergence_iterations = self.get_parameter('convergence_iterations').value
        self.signal_smooth_window = self.get_parameter('signal_smooth_window').value
        self.drone_id = self.get_parameter('drone_id').value
        
        # ===== 狀態變數 =====
        self.mode = OptimizerMode.SEARCH
        self.current_position = {'x': 0.0, 'y': 0.0, 'z': 2.0}
        
        # 最佳訊號記錄
        self.best_signal = {
            'snr': float('-inf'),
            'rssi': float('-inf'),
            'score': float('-inf'),
            'position': None,
            'timestamp': time.time()
        }
        
        # 收斂追蹤
        self.no_improvement_count = 0  # 連續無改進次數
        self.previous_best_score = float('-inf')
        
        # 訊號平滑 (降低噪聲)
        self.signal_history = []
        
        # 搜索統計
        self.search_count = 0
        self.search_start_time = time.time()
        
        # ===== 搜索範圍 =====
        if self.search_mode == 'fast':
            # 快速模式: 縮小搜索範圍
            self.search_bounds = {
                'x_min': -1.0, 'x_max': 1.0,
                'y_min': -1.0, 'y_max': 1.0,
                'z': 2.0
            }
            self.search_step = 0.4  # 較大的步長
            self.optimization_interval = 3.0  # 3秒更新一次 (加快)
            self.convergence_threshold = 0.5  # 0.5分以下視為無改進
        else:
            # 完整模式: 3x3 範圍
            self.search_bounds = {
                'x_min': -1.5, 'x_max': 1.5,
                'y_min': -1.5, 'y_max': 1.5,
                'z': 2.0
            }
            self.search_step = 0.3
            self.optimization_interval = 5.0
            self.convergence_threshold = 0.3
        
        # ===== ROS2 訂閱/發佈 =====
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
        
        # 發布優化狀態 (供 visualizer 和多機協調使用)
        self.status_pub = self.create_publisher(
            String,
            f'/{self.drone_id}/optimizer_status',
            10
        )
        
        # Vehicle status
        self.vehicle_status = None
        
        # ===== 定時器 =====
        self.optimization_timer = self.create_timer(
            self.optimization_interval,
            self.optimize_position
        )
        self.offboard_control_timer = self.create_timer(0.1, self.publish_offboard_mode)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info(
            f'Signal optimizer v2 ready | Mode: {self.mode.value} | '
            f'Drone: {self.drone_id} | Search: {self.search_mode}'
        )
    
    # ===== 訊號處理 =====
    def signal_callback(self, msg):
        """
        處理訊號品質數據 (JSON String from fast_scan_node)
        """
        try:
            data = json.loads(msg.data)
            
            if data.get('status') != 'Success':
                return
            
            current_snr = data.get('return_snr', float('-inf'))
            current_rssi = data.get('return_rssi', float('-inf'))
            
            # 訊號品質評分
            signal_score = current_snr * 0.7 + (current_rssi / -50.0) * 100 * 0.3
            
            # 訊號平滑 (移動平均)
            self.signal_history.append({
                'snr': current_snr,
                'rssi': current_rssi,
                'score': signal_score,
                'timestamp': time.time()
            })
            
            # 保持窗口大小
            if len(self.signal_history) > self.signal_smooth_window:
                self.signal_history.pop(0)
            
            # 使用平均值
            avg_score = np.mean([s['score'] for s in self.signal_history])
            
            # 檢查是否為新的最佳訊號
            self.check_and_update_best_signal(avg_score, current_snr, current_rssi)
            
        except (json.JSONDecodeError, Exception) as e:
            self.get_logger().debug(f'Signal processing error: {e}')
    
    def check_and_update_best_signal(self, score, snr, rssi):
        """檢查並更新最佳訊號記錄"""
        improvement = score - self.best_signal.get('score', float('-inf'))
        
        if improvement > self.convergence_threshold:
            # 有顯著改進
            self.best_signal = {
                'snr': snr,
                'rssi': rssi,
                'score': score,
                'position': self.current_position.copy(),
                'timestamp': time.time()
            }
            self.no_improvement_count = 0
            
            self.get_logger().info(
                f'✅ New best: SNR={snr:.1f}dB, RSSI={rssi:.1f}dBm, '
                f'Score={score:.1f} | Position: {self.best_signal["position"]}'
            )
        else:
            # 無顯著改進
            self.no_improvement_count += 1
            self.get_logger().debug(
                f'No improvement #{self.no_improvement_count}: '
                f'Current={score:.1f}, Best={self.best_signal.get("score", -999):.1f}'
            )
    
    # ===== 位置優化 =====
    def optimize_position(self):
        """執行位置優化邏輯"""
        if not self.vehicle_status or \
           self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return
        
        # 根據模式執行不同邏輯
        if self.mode == OptimizerMode.SEARCH:
            self.perform_search()
        elif self.mode == OptimizerMode.HOLD:
            self.hold_position()
        elif self.mode == OptimizerMode.COORDINATE:
            self.coordinate_with_others()
    
    def perform_search(self):
        """執行搜索"""
        # 檢查是否達到收斂條件
        if self.no_improvement_count >= self.convergence_iterations:
            self.mode = OptimizerMode.HOLD
            self.get_logger().info(
                f'🎯 收斂完成！切換到 HOLD 模式，停留在最佳點'
            )
            return
        
        # 生成下一個搜索位置
        new_position = self.get_next_search_position()
        if new_position:
            self.publish_trajectory_setpoint(
                new_position['x'],
                new_position['y'],
                new_position['z']
            )
            self.current_position = new_position
            self.search_count += 1
            
            elapsed = time.time() - self.search_start_time
            self.get_logger().debug(
                f'Search #{self.search_count} | Elapsed: {elapsed:.1f}s | '
                f'Position: ({new_position["x"]:.2f}, {new_position["y"]:.2f})'
            )
    
    def hold_position(self):
        """停留在最佳位置"""
        if self.best_signal['position']:
            pos = self.best_signal['position']
            self.publish_trajectory_setpoint(pos['x'], pos['y'], pos['z'])
            self.current_position = pos
            # 每秒記錄一次日誌
            if self.search_count % 5 == 0:
                self.get_logger().info(
                    f'⏸️  持續停留在最佳點: ({pos["x"]:.2f}, {pos["y"]:.2f}) | '
                    f'Best Score: {self.best_signal["score"]:.1f}'
                )
    
    def coordinate_with_others(self):
        """多機協調模式 (預留)"""
        # TODO: 實現多機協調邏輯
        # - 訂閱其他無人機的 /{drone_id}/optimizer_status
        # - 避免相互干擾
        # - 覆蓋不同區域
        pass
    
    def get_next_search_position(self):
        """生成下一個搜索位置 (隨機遊走)"""
        max_attempts = 10
        for _ in range(max_attempts):
            dx = np.random.uniform(-self.search_step, self.search_step)
            dy = np.random.uniform(-self.search_step, self.search_step)
            
            new_x = self.current_position['x'] + dx
            new_y = self.current_position['y'] + dy
            
            # 檢查邊界
            if (self.search_bounds['x_min'] <= new_x <= self.search_bounds['x_max'] and
                self.search_bounds['y_min'] <= new_y <= self.search_bounds['y_max']):
                return {
                    'x': round(new_x, 2),
                    'y': round(new_y, 2),
                    'z': self.search_bounds['z']
                }
        
        return None
    
    # ===== PX4 通訊 =====
    def vehicle_status_callback(self, msg):
        """處理飛行器狀態"""
        self.vehicle_status = msg
    
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
        msg.yaw = 0.0
        self.publisher_trajectory.publish(msg)
    
    # ===== 狀態發布 (多機用) =====
    def publish_status(self):
        """發布當前優化狀態 (供其他無人機訂閱)"""
        status = {
            'drone_id': self.drone_id,
            'mode': self.mode.value,
            'current_position': self.current_position,
            'best_signal': {
                'score': self.best_signal.get('score', -999),
                'position': self.best_signal.get('position'),
            },
            'search_count': self.search_count,
            'elapsed_time': time.time() - self.search_start_time,
            'timestamp': time.time()
        }
        
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SignalOptimizerNodeV2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
