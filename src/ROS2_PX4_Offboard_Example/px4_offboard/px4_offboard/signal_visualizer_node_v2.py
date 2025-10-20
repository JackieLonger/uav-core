#!/usr/bin/env python3
"""
Signal Visualizer Node v2 - 改進版
功能:
1. 訂閱 fast_scan_node 的 /link_quality (String JSON)
2. 訂閱 signal_optimizer 的狀態
3. 生成 RViz2 可視化:
   - 熱力圖 (訊號強度)
   - 飛行軌跡
   - 搜索邊界
   - 最佳點標記
4. 支援多機視覺化 (預留)
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, String
import numpy as np
import json
import colorsys


class SignalVisualizerNodeV2(Node):
    def __init__(self):
        super().__init__('signal_visualizer')
        
        # 參數
        self.declare_parameter('drone_id', 'drone_1')
        self.drone_id = self.get_parameter('drone_id').value
        
        # ===== 訂閱 =====
        # 訂閱訊號品質 (從 fast_scan_node)
        self.signal_sub = self.create_subscription(
            String,
            '/link_quality',
            self.signal_callback,
            10
        )
        
        # 訂閱優化器狀態 (從 signal_optimizer)
        self.optimizer_status_sub = self.create_subscription(
            String,
            f'/{self.drone_id}/optimizer_status',
            self.optimizer_status_callback,
            10
        )
        
        # ===== 發佈 =====
        self.heatmap_pub = self.create_publisher(
            MarkerArray,
            '/visualization/signal_heatmap',
            10
        )
        self.trajectory_pub = self.create_publisher(
            MarkerArray,
            f'/{self.drone_id}/trajectory',
            10
        )
        self.boundary_pub = self.create_publisher(
            MarkerArray,
            '/visualization/search_boundary',
            10
        )
        self.best_point_pub = self.create_publisher(
            Marker,
            f'/{self.drone_id}/best_point',
            10
        )
        self.current_position_pub = self.create_publisher(
            Marker,
            f'/{self.drone_id}/current_position',
            10
        )
        
        # ===== 資料儲存 =====
        self.signal_points = []  # [(x, y, z, score), ...]
        self.trajectory_points = []  # [(x, y, z), ...]
        self.best_position = None
        self.best_score = float('-inf')
        self.current_position = {'x': 0.0, 'y': 0.0, 'z': 2.0}
        
        # 搜索邊界
        self.search_bounds = {
            'x_min': -1.5, 'x_max': 1.5,
            'y_min': -1.5, 'y_max': 1.5,
            'z': 2.0
        }
        
        # 定時發布可視化
        self.create_timer(0.5, self.publish_visualization)
        
        self.get_logger().info(f'Signal visualizer v2 ready | Drone: {self.drone_id}')
    
    def signal_callback(self, msg):
        """接收訊號品質數據"""
        try:
            data = json.loads(msg.data)
            if data.get('status') != 'Success':
                return
            
            # 使用返回訊號 (更穩定)
            snr = data.get('return_snr', 0)
            rssi = data.get('return_rssi', 0)
            
            # 計算評分
            score = snr * 0.7 + (rssi / -50.0) * 100 * 0.3
            
            # 記錄點 (假設在最後的記錄位置)
            # 注: 實際應該由 optimizer 發送軌跡資訊
            self.signal_points.append({
                'x': self.current_position['x'],
                'y': self.current_position['y'],
                'z': self.current_position['z'],
                'score': score,
                'snr': snr,
                'rssi': rssi
            })
            
            # 限制儲存數量
            if len(self.signal_points) > 100:
                self.signal_points.pop(0)
            
        except Exception as e:
            self.get_logger().debug(f'Signal callback error: {e}')
    
    def optimizer_status_callback(self, msg):
        """接收優化器狀態"""
        try:
            data = json.loads(msg.data)
            
            # 更新當前位置
            pos = data.get('current_position', {})
            self.current_position = {
                'x': pos.get('x', 0.0),
                'y': pos.get('y', 0.0),
                'z': pos.get('z', 2.0)
            }
            self.trajectory_points.append(self.current_position.copy())
            
            # 限制軌跡點數
            if len(self.trajectory_points) > 200:
                self.trajectory_points.pop(0)
            
            # 更新最佳點
            best = data.get('best_signal', {})
            if best and best.get('position'):
                best_pos = best['position']
                self.best_position = best_pos
                self.best_score = best.get('score', -999)
            
        except Exception as e:
            self.get_logger().debug(f'Optimizer status callback error: {e}')
    
    def publish_visualization(self):
        """發布所有可視化"""
        # 1. 搜索邊界
        self.publish_search_boundary()
        
        # 2. 熱力圖
        self.publish_heatmap()
        
        # 3. 飛行軌跡
        self.publish_trajectory()
        
        # 4. 最佳點
        if self.best_position:
            self.publish_best_point()
        
        # 5. 當前位置
        self.publish_current_position()
    
    def publish_search_boundary(self):
        """發布搜索邊界 (3x3 方框)"""
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0
        marker.type = Marker.CUBE  # 立方體
        marker.action = Marker.ADD
        
        # 中心點
        center_x = (self.search_bounds['x_min'] + self.search_bounds['x_max']) / 2
        center_y = (self.search_bounds['y_min'] + self.search_bounds['y_max']) / 2
        
        marker.pose.position.x = center_x
        marker.pose.position.y = center_y
        marker.pose.position.z = self.search_bounds['z']
        marker.pose.orientation.w = 1.0
        
        # 尺寸
        marker.scale.x = self.search_bounds['x_max'] - self.search_bounds['x_min']
        marker.scale.y = self.search_bounds['y_max'] - self.search_bounds['y_min']
        marker.scale.z = 0.1
        
        # 顏色 (淡藍色透明)
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 0.2
        
        markers = MarkerArray()
        markers.markers.append(marker)
        self.boundary_pub.publish(markers)
    
    def publish_heatmap(self):
        """發布訊號熱力圖"""
        if not self.signal_points:
            return
        
        # 找最小最大評分 (用於色彩映射)
        scores = [p['score'] for p in self.signal_points]
        if not scores:
            return
        
        min_score = min(scores)
        max_score = max(scores)
        score_range = max_score - min_score if max_score > min_score else 1.0
        
        markers = MarkerArray()
        
        for idx, point in enumerate(self.signal_points):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = point['x']
            marker.pose.position.y = point['y']
            marker.pose.position.z = point['z']
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15
            
            # 色彩映射: 紅 (低) → 黃 → 綠 (高)
            normalized = (point['score'] - min_score) / score_range
            color = self.score_to_color(normalized)
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 0.7
            
            markers.markers.append(marker)
        
        self.heatmap_pub.publish(markers)
    
    def publish_trajectory(self):
        """發布飛行軌跡"""
        if not self.trajectory_points or len(self.trajectory_points) < 2:
            return
        
        # 軌跡線
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        for point in self.trajectory_points:
            p = Point()
            p.x = point['x']
            p.y = point['y']
            p.z = point['z']
            marker.points.append(p)
        
        marker.scale.x = 0.05  # 線寬
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        markers = MarkerArray()
        markers.markers.append(marker)
        self.trajectory_pub.publish(markers)
    
    def publish_best_point(self):
        """發布最佳點標記"""
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = self.best_position['x']
        marker.pose.position.y = self.best_position['y']
        marker.pose.position.z = self.best_position['z']
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        
        # 綠色發光
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.9
        
        self.best_point_pub.publish(marker)
    
    def publish_current_position(self):
        """發布當前位置 (無人機)"""
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = self.current_position['x']
        marker.pose.position.y = self.current_position['y']
        marker.pose.position.z = self.current_position['z']
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        
        # 藍色
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 0.9
        
        self.current_position_pub.publish(marker)
    
    @staticmethod
    def score_to_color(normalized_score):
        """
        將評分 (0-1) 映射到顏色
        0.0 (低) → 紅
        0.5 (中) → 黃
        1.0 (高) → 綠
        """
        # 使用 HSV 色彩模型
        # 紅(0°) → 黃(60°) → 綠(120°)
        hue = normalized_score * 120 / 360  # 轉換為 0-1
        saturation = 1.0
        value = 0.8
        
        r, g, b = colorsys.hsv_to_rgb(hue, saturation, value)
        return (r, g, b)


def main(args=None):
    rclpy.init(args=args)
    node = SignalVisualizerNodeV2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
