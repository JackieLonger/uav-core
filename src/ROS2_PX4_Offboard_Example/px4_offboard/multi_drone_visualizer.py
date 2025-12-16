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
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist, Point, PoseStamped
from std_msgs.msg import String, ColorRGBA, Header
from px4_msgs.msg import VehicleLocalPosition
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
import json
import math
import time
import csv
from collections import deque
from pathlib import Path as FilePath


class MultiDroneVisualizer(Node):
    """多无人机可视化节点"""
    
    def __init__(self):
        super().__init__('multi_drone_visualizer')
        
        # 参数 - 简化为只需要无人机数量
        self.declare_parameter('num_drones', 3)
        self.declare_parameter('drone_ids', [1, 2, 3])
        
        drone_ids = self.get_parameter('drone_ids').value
        
        # 每架无人机的状态
        self.drone_states = {}
        for drone_id in drone_ids:
            # Tracker ID 由 optimizer 动态识别，这里不需要预设
            binding = {
                'tracker_a': '!動態識別',
                'tracker_b': '!動態識別'
            }
            
            self.drone_states[drone_id] = {
                'position': Point(x=0.0, y=0.0, z=0.0),
                'quality': 0.0,
                'trajectory': deque(maxlen=100),  # 保留最近100个点
                'velocity': Twist(),
                'takeoff_position': None,
                'tracker_a_id': binding.get('tracker_a', '!未設定'),
                'tracker_b_id': binding.get('tracker_b', '!未設定'),
                # 信號歷史記錄（用於證明優化效果）
                'quality_history': deque(maxlen=1000),  # 最多記錄1000個數據點
                'rssi_history': deque(maxlen=1000),
                'snr_history': deque(maxlen=1000),
                'position_history': deque(maxlen=1000),
                'timestamp_history': deque(maxlen=1000),
                # 安全範圍數據（訂閱自 velocity_control.py）
                'safety_origin': None,
                'current_position_from_control': None,
                'safety_boundary_markers': [],
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
            
            # 安全範圍原點
            self.create_subscription(
                PoseStamped,
                f'/drone_{drone_id}/safety_origin',
                lambda msg, did=drone_id: self.safety_origin_callback(msg, did),
                10,
                callback_group=self.callback_group
            )
            
            # 當前位置（來自 velocity_control 的邊界檢查版本）
            self.create_subscription(
                PoseStamped,
                f'/drone_{drone_id}/current_position',
                lambda msg, did=drone_id: self.safety_position_callback(msg, did),
                10,
                callback_group=self.callback_group
            )
            
            # 安全邊界框
            self.create_subscription(
                MarkerArray,
                f'/drone_{drone_id}/safety_boundary',
                lambda msg, did=drone_id: self.safety_boundary_callback(msg, did),
                10,
                callback_group=self.callback_group
            )
        
        # 发布 MarkerArray
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/visualization_marker_array',
            10
        )
        
        # 發布每架無人機的軌跡 Path
        self.path_pubs = {}
        for drone_id in drone_ids:
            self.path_pubs[drone_id] = self.create_publisher(
                Path,
                f'/drone_{drone_id}/path',
                10
            )
        
        # 創建日誌目錄用於保存信號歷史
        self.log_dir = FilePath.home() / 'uav-core' / 'signal_logs'
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f"信號日誌將保存到: {self.log_dir}")
        
        # 定时发布可视化
        self.timer = self.create_timer(
            0.1,  # 10Hz
            self.publish_visualization,
            callback_group=self.callback_group
        )
        
        self.get_logger().info(f"多无人机可视化器启动（{len(drone_ids)} 架）")
    
    def position_callback(self, msg: VehicleLocalPosition, drone_id: int):
        """接收位置数据 (NED → ENU 座標轉換)"""
        state = self.drone_states[drone_id]
        
        # NED to ENU 座標轉換 (用於 RViz2 顯示)
        # NED: X=North, Y=East, Z=Down
        # ENU: X=East, Y=North, Z=Up
        # 正確轉換公式:
        #   ENU_X = NED_Y (East)
        #   ENU_Y = NED_X (North)
        #   ENU_Z = -NED_Z (Up = -Down)
        
        # 修正：如果多台無人機重疊，可能是因為它們的相對位置沒有正確反映
        # 假設每台無人機的 local_position 都是相對於它自己的起飛點 (0,0,0)
        # 我們需要加上它們的初始偏移量（如果有的話）
        # 但在實際實驗中，我們通常希望看到它們相對於起飛點的移動
        
        pos = Point(
            x=msg.y,      # ENU_X = NED_Y (East)
            y=msg.x,      # ENU_Y = NED_X (North)
            z=-msg.z      # ENU_Z = -NED_Z (Up)
        )
        
        # 如果是第一台無人機，保持原點
        # 如果是第二/三台，手動偏移顯示（僅用於視覺化區分，不影響控制）
        # 使用較小偏移量 (0.5m) 以更好顯示實際移動
        if drone_id == 2:
            pos.x += 0.5  # 向東偏移 0.5 米
        elif drone_id == 3:
            pos.x -= 0.5  # 向西偏移 0.5 米
            
        state['position'] = pos
        state['trajectory'].append(pos)
        
        # 记录起飞位置
        if state['takeoff_position'] is None:
            state['takeoff_position'] = pos
            self.get_logger().info(f'Drone {drone_id} 起飛位置: ({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f})')
        
        # 記錄位置歷史 (使用 ENU 座標)
        timestamp = time.time()
        state['position_history'].append([pos.x, pos.y, pos.z])
        state['timestamp_history'].append(timestamp)
        
        # 調試輸出 (前3個位置)
        if len(state['position_history']) <= 3:
            self.get_logger().info(
                f'Drone {drone_id} 位置 #{len(state["position_history"])}: '
                f'NED({msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f}) → '
                f'ENU({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f})'
            )
        
        # 發布軌跡 (Path message for RViz2)
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        # 從軌跡創建 Path
        for p in state['trajectory']:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position = p
            pose.pose.orientation.w = 1.0  # Identity quaternion
            path_msg.poses.append(pose)
        
        self.path_pubs[drone_id].publish(path_msg)
    
    def quality_callback(self, msg: String, drone_id: int):
        """接收信号质量数据"""
        try:
            data = json.loads(msg.data)
            
            # 计算综合质量
            if data.get('status') == 'Success':
                # 安全的類型轉換 (JSON 值可能是字串)
                forward_rssi = float(data.get('forward_rssi', 0.0) or 0.0)
                forward_snr = float(data.get('forward_snr', 0.0) or 0.0)
                return_rssi = float(data.get('return_rssi', 0.0) or 0.0)
                return_snr = float(data.get('return_snr', 0.0) or 0.0)
                
                # RSSI 归一化 [-120, -30] dBm → [0, 1]
                rssi_avg = (forward_rssi + return_rssi) / 2.0
                rssi_score = (rssi_avg + 120) / 90.0  # -120 dBm = 0, -30 dBm = 1
                rssi_score = max(0.0, min(1.0, rssi_score))
                
                # SNR 归一化 [-20, 15] dB → [0, 1]
                snr_avg = (forward_snr + return_snr) / 2.0
                snr_score = (snr_avg + 20) / 35.0  # -20 dB = 0, 15 dB = 1
                snr_score = max(0.0, min(1.0, snr_score))
                
                # 综合评分 (0.5 RSSI + 0.5 SNR)
                quality = 0.5 * rssi_score + 0.5 * snr_score
                
                # 直接更新 (不要平均，使用最新值)
                state = self.drone_states[drone_id]
                state['quality'] = quality
                
                # 記錄信號質量歷史
                timestamp = time.time()
                state['quality_history'].append(quality)
                state['rssi_history'].append(rssi_avg)
                state['snr_history'].append(snr_avg)
                state['timestamp_history'].append(timestamp)  # 確保同步記錄時間戳
                
                # 調試輸出
                if len(state['quality_history']) <= 5 or len(state['quality_history']) % 10 == 0:
                    self.get_logger().info(f'Drone {drone_id} 信號記錄 #{len(state["quality_history"])}: Q={quality:.2f}, RSSI={rssi_avg:.1f}dBm, SNR={snr_avg:.1f}dB')
                
        except Exception as e:
            self.get_logger().error(f"质量回调错误: {e}")
    
    def velocity_callback(self, msg: Twist, drone_id: int):
        """接收速度指令"""
        self.drone_states[drone_id]['velocity'] = msg
    
    def safety_origin_callback(self, msg: PoseStamped, drone_id: int):
        """接收安全範圍原點"""
        state = self.drone_states[drone_id]
        state['safety_origin'] = Point(
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            z=msg.pose.position.z
        )
    
    def safety_position_callback(self, msg: PoseStamped, drone_id: int):
        """接收當前位置（來自邊界檢查）"""
        state = self.drone_states[drone_id]
        state['current_position_from_control'] = Point(
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            z=msg.pose.position.z
        )
    
    def safety_boundary_callback(self, msg: MarkerArray, drone_id: int):
        """接收安全邊界框"""
        state = self.drone_states[drone_id]
        state['safety_boundary_markers'] = msg.markers if msg.markers else []
    
    def publish_visualization(self):
        """发布可视化标记"""
        markers = MarkerArray()
        marker_id = 0
        
        # 注释: 不再显示固定的地面 Tracker 位置
        # 因为每架无人机绑定不同的 Tracker
        # Tracker 信息显示在无人机标签上
        
        # ========== 首先添加來自 velocity_control 的安全邊界框 ==========
        for drone_id, state in self.drone_states.items():
            if state['safety_boundary_markers']:
                for marker in state['safety_boundary_markers']:
                    marker.id = marker_id
                    marker.ns = f"safety_boundary_drone_{drone_id}"
                    markers.markers.append(marker)
                    marker_id += 1
        
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
                
                # 定义立方体的 8 个顶点（ENU 座標系）
                # takeoff 已經是 ENU 座標 (在 position_callback 中轉換)
                # ENU: X=East, Y=North, Z=Up
                vertices = [
                    # 底部 4 个顶点（z = takeoff.z，起飛高度）
                    Point(x=takeoff.x - 1.5, y=takeoff.y - 1.5, z=takeoff.z),
                    Point(x=takeoff.x + 1.5, y=takeoff.y - 1.5, z=takeoff.z),
                    Point(x=takeoff.x + 1.5, y=takeoff.y + 1.5, z=takeoff.z),
                    Point(x=takeoff.x - 1.5, y=takeoff.y + 1.5, z=takeoff.z),
                    # 顶部 4 个顶点（z = takeoff.z + 3.0，向上 3 米）
                    Point(x=takeoff.x - 1.5, y=takeoff.y - 1.5, z=takeoff.z + 3.0),
                    Point(x=takeoff.x + 1.5, y=takeoff.y - 1.5, z=takeoff.z + 3.0),
                    Point(x=takeoff.x + 1.5, y=takeoff.y + 1.5, z=takeoff.z + 3.0),
                    Point(x=takeoff.x - 1.5, y=takeoff.y + 1.5, z=takeoff.z + 3.0),
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
            
            # 2.6 RSSI 歷史曲線 (紅色線條)
            rssi_history = state['rssi_history']
            if len(rssi_history) >= 2:
                rssi_line = Marker()
                rssi_line.header.frame_id = "map"
                rssi_line.header.stamp = self.get_clock().now().to_msg()
                rssi_line.ns = f"rssi_history_{drone_id}"
                rssi_line.id = marker_id
                marker_id += 1
                rssi_line.type = Marker.LINE_STRIP
                rssi_line.action = Marker.ADD
                rssi_line.scale.x = 0.05  # 線條寬度
                rssi_line.color.r = 1.0
                rssi_line.color.g = 0.2
                rssi_line.color.b = 0.2
                rssi_line.color.a = 0.8
                
                # 取最近 50 個數據點
                recent_rssi = rssi_history[-50:] if len(rssi_history) > 50 else rssi_history
                for i, rssi in enumerate(recent_rssi):
                    pt = Point()
                    pt.x = pos.x + (i - len(recent_rssi)/2) * 0.1  # 水平展開
                    pt.y = pos.y + 1.0  # Y 方向偏移
                    # RSSI 歸一化到高度 [-120, -30] dBm → [0, 2] m
                    pt.z = pos.z + 2.0 + (rssi + 120) / 45.0
                    rssi_line.points.append(pt)
                
                markers.markers.append(rssi_line)
            
            # 2.7 SNR 歷史曲線 (藍色線條)
            snr_history = state['snr_history']
            if len(snr_history) >= 2:
                snr_line = Marker()
                snr_line.header.frame_id = "map"
                snr_line.header.stamp = self.get_clock().now().to_msg()
                snr_line.ns = f"snr_history_{drone_id}"
                snr_line.id = marker_id
                marker_id += 1
                snr_line.type = Marker.LINE_STRIP
                snr_line.action = Marker.ADD
                snr_line.scale.x = 0.05
                snr_line.color.r = 0.2
                snr_line.color.g = 0.2
                snr_line.color.b = 1.0
                snr_line.color.a = 0.8
                
                # 取最近 50 個數據點
                recent_snr = snr_history[-50:] if len(snr_history) > 50 else snr_history
                for i, snr in enumerate(recent_snr):
                    pt = Point()
                    pt.x = pos.x + (i - len(recent_snr)/2) * 0.1
                    pt.y = pos.y - 1.0  # Y 方向偏移 (與 RSSI 相反)
                    # SNR 歸一化到高度 [-20, 15] dB → [0, 2] m
                    pt.z = pos.z + 2.0 + (snr + 20) / 17.5
                    snr_line.points.append(pt)
                
                markers.markers.append(snr_line)
            
            # 2.8 信號數值標籤
            if len(rssi_history) > 0 and len(snr_history) > 0:
                signal_label = Marker()
                signal_label.header.frame_id = "map"
                signal_label.header.stamp = self.get_clock().now().to_msg()
                signal_label.ns = f"signal_label_{drone_id}"
                signal_label.id = marker_id
                marker_id += 1
                signal_label.type = Marker.TEXT_VIEW_FACING
                signal_label.action = Marker.ADD
                signal_label.pose.position.x = pos.x
                signal_label.pose.position.y = pos.y
                signal_label.pose.position.z = pos.z + 4.0  # 無人機上方 4m
                signal_label.scale.z = 0.3
                signal_label.color.r = 1.0
                signal_label.color.g = 1.0
                signal_label.color.b = 1.0
                signal_label.color.a = 1.0
                signal_label.text = f"RSSI:{rssi_history[-1]:.0f}dBm SNR:{snr_history[-1]:.1f}dB"
                markers.markers.append(signal_label)
        
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
    
    def save_signal_history(self):
        """保存信號質量歷史到 CSV"""
        try:
            print(f"\n[DEBUG] 開始保存信號歷史...")
            for drone_id, state in self.drone_states.items():
                print(f"[DEBUG] Drone {drone_id}: {len(state['timestamp_history'])} 個時間戳, {len(state['quality_history'])} 個質量數據")
                
                # 檢查是否有數據
                if len(state['timestamp_history']) == 0:
                    print(f"[DEBUG] Drone {drone_id} 沒有數據，跳過")
                    continue
                
                # 生成文件名
                timestamp_str = time.strftime('%Y%m%d_%H%M%S')
                filename = self.log_dir / f'signal_log_drone{drone_id}_{timestamp_str}.csv'
                print(f"[DEBUG] 保存到: {filename}")
                
                # 寫入 CSV
                with open(filename, 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    
                    # 表頭
                    writer.writerow([
                        'timestamp', 'drone_id', 'quality', 'rssi', 'snr', 
                        'pos_x', 'pos_y', 'pos_z'
                    ])
                    
                    # 數據行
                    for i in range(len(state['timestamp_history'])):
                        timestamp = state['timestamp_history'][i]
                        quality = state['quality_history'][i] if i < len(state['quality_history']) else 0.0
                        rssi = state['rssi_history'][i] if i < len(state['rssi_history']) else 0.0
                        snr = state['snr_history'][i] if i < len(state['snr_history']) else 0.0
                        
                        if i < len(state['position_history']):
                            pos = state['position_history'][i]
                            pos_x, pos_y, pos_z = pos[0], pos[1], pos[2]
                        else:
                            pos_x, pos_y, pos_z = 0.0, 0.0, 0.0
                        
                        writer.writerow([
                            timestamp, drone_id, quality, rssi, snr,
                            pos_x, pos_y, pos_z
                        ])
                
                self.get_logger().info(f'信號歷史已保存: {filename}')
                
        except Exception as e:
            self.get_logger().error(f'保存信號歷史失敗: {e}')
    
    def __del__(self):
        """析構函數 - 節點關閉時保存數據"""
        try:
            self.save_signal_history()
        except:
            pass  # 避免析構時出錯


def main(args=None):
    rclpy.init(args=args)
    node = MultiDroneVisualizer()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # 正常關閉
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # 保存信號歷史
        try:
            node.get_logger().info("可视化器关闭")
            node.save_signal_history()
        except:
            pass
        
        try:
            node.destroy_node()
        except:
            pass
        
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
