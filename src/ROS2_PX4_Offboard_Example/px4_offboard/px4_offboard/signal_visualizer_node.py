#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from meshtastic_ros_bridge_msgs.msg import MeshSignalQuality
import numpy as np

class SignalVisualizerNode(Node):
    def __init__(self):
        super().__init__('signal_visualizer')
        
        # 訂閱訊號品質
        self.signal_sub = self.create_subscription(
            MeshSignalQuality,
            '/meshtastic/link_quality',
            self.signal_callback,
            10
        )
        
        # 發布視覺化標記
        self.heatmap_pub = self.create_publisher(
            MarkerArray,
            '/visualization/signal_heatmap',
            10
        )
        self.boundary_pub = self.create_publisher(
            MarkerArray,
            '/visualization/search_boundary',
            10
        )
        self.trajectory_pub = self.create_publisher(
            MarkerArray,
            '/drone1/trajectory',
            10
        )
        self.drone_pub = self.create_publisher(
            Marker,
            '/drone1/model',
            10
        )
        
        # 訊號數據儲存
        self.signal_points = []
        self.trajectory_points = []
        
        # 定期更新視覺化
        self.create_timer(1.0, self.update_visualization)
        
        # 發布搜索邊界
        self.publish_search_boundary()
        
    def signal_callback(self, msg):
        """處理訊號品質數據"""
        try:
            # 儲存訊號數據點
            point = {
                'x': msg.position.x,
                'y': msg.position.y,
                'z': msg.position.z,
                'snr': msg.return_snr,
                'rssi': msg.return_rssi
            }
            self.signal_points.append(point)
            self.trajectory_points.append(Point(x=point['x'], y=point['y'], z=point['z']))
        except Exception as e:
            self.get_logger().error(f'Error processing signal data: {e}')
    
    def update_visualization(self):
        """更新視覺化"""
        self.publish_signal_heatmap()
        self.publish_trajectory()
        self.publish_drone_model()
    
    def publish_signal_heatmap(self):
        """發布訊號熱力圖"""
        marker_array = MarkerArray()
        
        if not self.signal_points:
            return
        
        # 創建網格點
        resolution = 0.2  # meters
        x_min, x_max = -1.5, 1.5
        y_min, y_max = -1.5, 1.5
        
        for x in np.arange(x_min, x_max, resolution):
            for y in np.arange(y_min, y_max, resolution):
                # 計算這點的訊號強度（使用反距離加權）
                total_weight = 0
                weighted_snr = 0
                
                for point in self.signal_points:
                    dist = np.sqrt((x - point['x'])**2 + (y - point['y'])**2)
                    if dist < 0.001:
                        dist = 0.001
                    weight = 1 / dist**2
                    total_weight += weight
                    weighted_snr += weight * point['snr']
                
                if total_weight > 0:
                    avg_snr = weighted_snr / total_weight
                    
                    # 創建標記
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    marker.pose.position.x = x
                    marker.pose.position.y = y
                    marker.pose.position.z = 0.1
                    marker.scale.x = resolution
                    marker.scale.y = resolution
                    marker.scale.z = 0.1
                    
                    # 根據 SNR 設置顏色
                    color = self.get_color_from_snr(avg_snr)
                    marker.color = color
                    
                    marker_array.markers.append(marker)
        
        # 設置標記 ID
        for id, marker in enumerate(marker_array.markers):
            marker.id = id
        
        self.heatmap_pub.publish(marker_array)
    
    def publish_trajectory(self):
        """發布無人機軌跡"""
        if not self.trajectory_points:
            return
            
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # line width
        marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        marker.points = self.trajectory_points
        
        marker_array = MarkerArray()
        marker_array.markers = [marker]
        self.trajectory_pub.publish(marker_array)
    
    def publish_drone_model(self):
        """發布無人機模型"""
        if not self.trajectory_points:
            return
            
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = "package://ros2_px4_offboard_example/meshes/quad.dae"
        marker.action = Marker.ADD
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        
        # 使用最新位置
        latest_point = self.trajectory_points[-1]
        marker.pose.position = latest_point
        
        self.drone_pub.publish(marker)
    
    def publish_search_boundary(self):
        """發布搜索邊界"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # line width
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        
        # 建立邊界點
        points = [
            Point(x=-1.5, y=-1.5, z=0.0),
            Point(x=1.5, y=-1.5, z=0.0),
            Point(x=1.5, y=1.5, z=0.0),
            Point(x=-1.5, y=1.5, z=0.0),
            Point(x=-1.5, y=-1.5, z=0.0)
        ]
        marker.points = points
        
        marker_array = MarkerArray()
        marker_array.markers = [marker]
        self.boundary_pub.publish(marker_array)
    
    def get_color_from_snr(self, snr):
        """根據 SNR 生成顏色"""
        # SNR 範圍通常在 -20 到 20 之間
        min_snr = -20
        max_snr = 20
        
        # 將 SNR 正規化到 0-1 範圍
        normalized = (snr - min_snr) / (max_snr - min_snr)
        normalized = max(0, min(1, normalized))
        
        # 使用紅到綠的漸變
        color = ColorRGBA()
        color.r = 1.0 - normalized
        color.g = normalized
        color.b = 0.0
        color.a = 0.7
        
        return color

def main(args=None):
    rclpy.init(args=args)
    node = SignalVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()