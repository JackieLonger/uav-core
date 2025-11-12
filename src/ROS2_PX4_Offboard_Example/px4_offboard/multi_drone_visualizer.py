#!/usr/bin/env python3
"""
RViz2 可视化节点 - 多无人机信号优化系统

显示内容：
1. 每架无人机的位置（彩色球体）
2. 信号强度（颜色渐变：红→黄→绿）
3. 运动轨迹（历史路径线）
4. 搜索边界（立方体框架）
5. 地面信标位置（柱形）
6. 当前质量分数（文本标签）
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String, ColorRGBA
from px4_msgs.msg import VehicleLocalPosition
from visualization_msgs.msg import Marker, MarkerArray
import json
import math
from collections import deque


class MultiDroneVisualizer(Node):
    """多无人机可视化节点"""
    
    def __init__(self):
        super().__init__('multi_drone_visualizer')
        
        # 参数
        self.declare_parameter('num_drones', 3)
        self.declare_parameter('drone_ids', [1, 2, 3])
        self.declare_parameter('drone_tracker_bindings', {
            '1': {'tracker_a': '!待設定1A', 'tracker_b': '!待設定1B'},
            '2': {'tracker_a': '!待設定2A', 'tracker_b': '!待設定2B'},
            '3': {'tracker_a': '!待設定3A', 'tracker_b': '!待設定3B'}
        })
        
        drone_ids = self.get_parameter('drone_ids').value
        drone_tracker_bindings = self.get_parameter('drone_tracker_bindings').value
        
        # 每架无人机的状态
        self.drone_states = {}
        for drone_id in drone_ids:
            # 获取该无人机绑定的 Tracker
            binding = drone_tracker_bindings.get(str(drone_id), {
                'tracker_a': '!未設定',
                'tracker_b': '!未設定'
            })
            
            self.drone_states[drone_id] = {
                'position': Point(x=0.0, y=0.0, z=0.0),
                'quality': 0.0,
                'trajectory': deque(maxlen=100),  # 保留最近100个点
                'velocity': Twist(),
                'takeoff_position': None,
                'tracker_a_id': binding.get('tracker_a', '!未設定'),
                'tracker_b_id': binding.get('tracker_b', '!未設定'),
            }
        
        self.get_logger().info(f"可视化器启动，绑定信息：")
        for drone_id, state in self.drone_states.items():
            self.get_logger().info(
                f"  Drone {drone_id}: {state['tracker_a_id']} & {state['tracker_b_id']}"
            )
        
        # 回调组
        self.callback_group = ReentrantCallbackGroup()
        
        # 订阅每架无人机的数据
        for drone_id in drone_ids:
            # 位置
            self.create_subscription(
                VehicleLocalPosition,
                f'/drone_{drone_id}/fmu/out/vehicle_local_position',
                lambda msg, did=drone_id: self.position_callback(msg, did),
                10,
                callback_group=self.callback_group
            )
            
            # 信号质量
            self.create_subscription(
                String,
                f'/drone_{drone_id}/link_quality',
                lambda msg, did=drone_id: self.quality_callback(msg, did),
                10,
                callback_group=self.callback_group
            )
            
            # 速度指令
            self.create_subscription(
                Twist,
                f'/drone_{drone_id}/offboard_velocity_cmd',
                lambda msg, did=drone_id: self.velocity_callback(msg, did),
                10,
                callback_group=self.callback_group
            )
        
        # 发布 MarkerArray
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/visualization_marker_array',
            10
        )
        
        # 定时发布可视化
        self.timer = self.create_timer(
            0.1,  # 10Hz
            self.publish_visualization,
            callback_group=self.callback_group
        )
        
        self.get_logger().info(f"多无人机可视化器启动（{len(drone_ids)} 架）")
    
    def position_callback(self, msg: VehicleLocalPosition, drone_id: int):
        """接收位置数据"""
        state = self.drone_states[drone_id]
        pos = Point(x=msg.x, y=msg.y, z=msg.z)
        state['position'] = pos
        state['trajectory'].append(pos)
        
        # 记录起飞位置
        if state['takeoff_position'] is None:
            state['takeoff_position'] = pos
    
    def quality_callback(self, msg: String, drone_id: int):
        """接收信号质量数据"""
        try:
            data = json.loads(msg.data)
            
            # 计算综合质量
            if data.get('status') == 'Success':
                forward_rssi = data.get('forward_rssi', 0.0)
                forward_snr = data.get('forward_snr', 0.0)
                return_rssi = data.get('return_rssi', 0.0)
                return_snr = data.get('return_snr', 0.0)
                
                # RSSI 归一化
                rssi_avg = (forward_rssi + return_rssi) / 2.0
                rssi_score = (rssi_avg + 100) / 80.0
                rssi_score = max(0.0, min(1.0, rssi_score))
                
                # SNR 归一化
                snr_avg = (forward_snr + return_snr) / 2.0
                snr_score = (snr_avg + 10) / 30.0
                snr_score = max(0.0, min(1.0, snr_score))
                
                # 综合评分
                quality = 0.5 * rssi_score + 0.5 * snr_score
                
                # 更新（如果有两个tracker，取平均）
                state = self.drone_states[drone_id]
                if state['quality'] == 0.0:
                    state['quality'] = quality
                else:
                    state['quality'] = (state['quality'] + quality) / 2.0
                
        except Exception as e:
            self.get_logger().error(f"质量回调错误: {e}")
    
    def velocity_callback(self, msg: Twist, drone_id: int):
        """接收速度指令"""
        self.drone_states[drone_id]['velocity'] = msg
    
    def publish_visualization(self):
        """发布可视化标记"""
        markers = MarkerArray()
        marker_id = 0
        
        # 注释: 不再显示固定的地面 Tracker 位置
        # 因为每架无人机绑定不同的 Tracker
        # Tracker 信息显示在无人机标签上
        
        # 开始显示每架无人机
        for drone_id, state in self.drone_states.items():
            pos = state['position']
            quality = state['quality']
            
            # 2.1 无人机位置（球体）
            drone_marker = Marker()
            drone_marker.header.frame_id = "map"
            drone_marker.header.stamp = self.get_clock().now().to_msg()
            drone_marker.ns = "drones"
            drone_marker.id = marker_id
            marker_id += 1
            drone_marker.type = Marker.SPHERE
            drone_marker.action = Marker.ADD
            
            drone_marker.pose.position = pos
            drone_marker.pose.orientation.w = 1.0
            
            drone_marker.scale.x = 0.5
            drone_marker.scale.y = 0.5
            drone_marker.scale.z = 0.5
            
            # 颜色基于质量：红(0) → 黄(0.5) → 绿(1.0)
            color = self.quality_to_color(quality)
            drone_marker.color = color
            
            markers.markers.append(drone_marker)
            
            # 2.2 无人机ID文本
            id_marker = Marker()
            id_marker.header.frame_id = "map"
            id_marker.header.stamp = self.get_clock().now().to_msg()
            id_marker.ns = "drone_ids"
            id_marker.id = marker_id
            marker_id += 1
            id_marker.type = Marker.TEXT_VIEW_FACING
            id_marker.action = Marker.ADD
            
            id_marker.pose.position.x = pos.x
            id_marker.pose.position.y = pos.y
            id_marker.pose.position.z = pos.z + 0.8
            
            id_marker.scale.z = 0.3
            id_marker.color.r = 1.0
            id_marker.color.g = 1.0
            id_marker.color.b = 1.0
            id_marker.color.a = 1.0
            
            # 显示无人机编号、质量和绑定的 Tracker
            tracker_a = state['tracker_a_id']
            tracker_b = state['tracker_b_id']
            id_marker.text = f"Drone {drone_id}\nQ: {quality:.2f}\n绑定: {tracker_a}\n      {tracker_b}"
            markers.markers.append(id_marker)
            
            # 2.3 运动轨迹
            if len(state['trajectory']) > 1:
                traj_marker = Marker()
                traj_marker.header.frame_id = "map"
                traj_marker.header.stamp = self.get_clock().now().to_msg()
                traj_marker.ns = "trajectories"
                traj_marker.id = marker_id
                marker_id += 1
                traj_marker.type = Marker.LINE_STRIP
                traj_marker.action = Marker.ADD
                
                traj_marker.scale.x = 0.05  # 线宽
                
                # 轨迹颜色（半透明）
                traj_marker.color.r = 0.0
                traj_marker.color.g = 0.7
                traj_marker.color.b = 1.0
                traj_marker.color.a = 0.6
                
                traj_marker.points = list(state['trajectory'])
                markers.markers.append(traj_marker)
            
            # 2.4 搜索边界（每架无人机独立的 3m³ 边界）
            if state['takeoff_position'] is not None:
                # 使用 LINE_LIST 画框架线，更清晰
                bounds_marker = Marker()
                bounds_marker.header.frame_id = "map"
                bounds_marker.header.stamp = self.get_clock().now().to_msg()
                bounds_marker.ns = f"search_bounds_drone_{drone_id}"
                bounds_marker.id = marker_id
                marker_id += 1
                bounds_marker.type = Marker.LINE_LIST
                bounds_marker.action = Marker.ADD
                
                takeoff = state['takeoff_position']
                
                # 定义立方体的 8 个顶点（相对于 takeoff_position）
                # NED: x前, y右, z下
                vertices = [
                    # 底部 4 个顶点（z = takeoff.z，起飞高度）
                    Point(x=takeoff.x - 1.5, y=takeoff.y - 1.5, z=takeoff.z),
                    Point(x=takeoff.x + 1.5, y=takeoff.y - 1.5, z=takeoff.z),
                    Point(x=takeoff.x + 1.5, y=takeoff.y + 1.5, z=takeoff.z),
                    Point(x=takeoff.x - 1.5, y=takeoff.y + 1.5, z=takeoff.z),
                    # 顶部 4 个顶点（z = takeoff.z - 3.0，上升3m）
                    Point(x=takeoff.x - 1.5, y=takeoff.y - 1.5, z=takeoff.z - 3.0),
                    Point(x=takeoff.x + 1.5, y=takeoff.y - 1.5, z=takeoff.z - 3.0),
                    Point(x=takeoff.x + 1.5, y=takeoff.y + 1.5, z=takeoff.z - 3.0),
                    Point(x=takeoff.x - 1.5, y=takeoff.y + 1.5, z=takeoff.z - 3.0),
                ]
                
                # 定义立方体的 12 条边（每条边需要 2 个点）
                edges = [
                    # 底部 4 条边
                    (0, 1), (1, 2), (2, 3), (3, 0),
                    # 顶部 4 条边
                    (4, 5), (5, 6), (6, 7), (7, 4),
                    # 垂直 4 条边
                    (0, 4), (1, 5), (2, 6), (3, 7),
                ]
                
                # 添加所有边的点
                for edge in edges:
                    bounds_marker.points.append(vertices[edge[0]])
                    bounds_marker.points.append(vertices[edge[1]])
                
                bounds_marker.scale.x = 0.05  # 线宽
                
                # 每架无人机不同颜色的边界
                drone_colors = [
                    (0.0, 1.0, 1.0),  # Drone 1: 青色
                    (1.0, 0.0, 1.0),  # Drone 2: 洋红
                    (1.0, 1.0, 0.0),  # Drone 3: 黄色
                ]
                color_idx = (drone_id - 1) % len(drone_colors)
                bounds_marker.color.r = drone_colors[color_idx][0]
                bounds_marker.color.g = drone_colors[color_idx][1]
                bounds_marker.color.b = drone_colors[color_idx][2]
                bounds_marker.color.a = 0.6  # 半透明
                
                markers.markers.append(bounds_marker)
                
                # 添加边界中心标签
                center_marker = Marker()
                center_marker.header.frame_id = "map"
                center_marker.header.stamp = self.get_clock().now().to_msg()
                center_marker.ns = f"bounds_label_drone_{drone_id}"
                center_marker.id = marker_id
                marker_id += 1
                center_marker.type = Marker.TEXT_VIEW_FACING
                center_marker.action = Marker.ADD
                
                center_marker.pose.position.x = takeoff.x
                center_marker.pose.position.y = takeoff.y
                center_marker.pose.position.z = takeoff.z - 1.5  # 边界中心
                
                center_marker.scale.z = 0.25
                center_marker.color.r = drone_colors[color_idx][0]
                center_marker.color.g = drone_colors[color_idx][1]
                center_marker.color.b = drone_colors[color_idx][2]
                center_marker.color.a = 0.8
                
                center_marker.text = f"Drone {drone_id}\n搜索范围"
                markers.markers.append(center_marker)
            
            # 2.5 速度向量箭头
            vel = state['velocity']
            vel_mag = math.sqrt(vel.linear.x**2 + vel.linear.y**2 + vel.linear.z**2)
            if vel_mag > 0.01:
                arrow_marker = Marker()
                arrow_marker.header.frame_id = "map"
                arrow_marker.header.stamp = self.get_clock().now().to_msg()
                arrow_marker.ns = "velocities"
                arrow_marker.id = marker_id
                marker_id += 1
                arrow_marker.type = Marker.ARROW
                arrow_marker.action = Marker.ADD
                
                # 箭头起点
                arrow_marker.points.append(pos)
                
                # 箭头终点
                end_point = Point()
                end_point.x = pos.x + vel.linear.x * 2.0  # 放大2倍显示
                end_point.y = pos.y + vel.linear.y * 2.0
                end_point.z = pos.z + vel.linear.z * 2.0
                arrow_marker.points.append(end_point)
                
                arrow_marker.scale.x = 0.1  # 箭杆宽度
                arrow_marker.scale.y = 0.2  # 箭头宽度
                
                arrow_marker.color.r = 1.0
                arrow_marker.color.g = 1.0
                arrow_marker.color.b = 0.0
                arrow_marker.color.a = 0.8
                
                markers.markers.append(arrow_marker)
        
        # 发布
        self.marker_pub.publish(markers)
    
    def quality_to_color(self, quality: float) -> ColorRGBA:
        """
        将质量分数转换为颜色
        0.0 (差) → 红色
        0.5 (中) → 黄色
        1.0 (好) → 绿色
        """
        color = ColorRGBA()
        
        if quality < 0.5:
            # 红 → 黄
            ratio = quality / 0.5
            color.r = 1.0
            color.g = ratio
            color.b = 0.0
        else:
            # 黄 → 绿
            ratio = (quality - 0.5) / 0.5
            color.r = 1.0 - ratio
            color.g = 1.0
            color.b = 0.0
        
        color.a = 0.8
        return color


def main(args=None):
    rclpy.init(args=args)
    node = MultiDroneVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("可视化器关闭")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
