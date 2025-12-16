#!/usr/bin/env python3
"""
Jetson 机载节点启动文件

用法（在每架无人机的 Jetson 上运行）：
    ros2 launch ros2_px4_offboard_example jetson_onboard.launch.py drone_id:=1
    ros2 launch ros2_px4_offboard_example jetson_onboard.launch.py drone_id:=2
    ros2 launch ros2_px4_offboard_example jetson_onboard.launch.py drone_id:=3

功能：
- 启动 fast_scan_node（扫描 Meshtastic）
- 启动 velocity_control（接收速度指令 → 发给 Pixhawk）
- 自动配置正确的 topic 名称

注意：
- Tracker ID 已在 fast_scan_node.py 中硬编码
- 每架无人机的 Jetson 需手动修改 fast_scan_node.py 的配置区域
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """动态生成 launch actions（在运行时解析 LaunchConfiguration）"""
    
    # 获取 drone_id 的实际值
    drone_id = LaunchConfiguration('drone_id').perform(context)
    
    # 1. fast_scan_node（扫描 Meshtastic）
    fast_scan_node = Node(
        package='ros2_px4_offboard_example',
        executable='fast_scan_node.py',
        name=f'fast_scan_drone_{drone_id}',
        output='screen',
        emulate_tty=True,
        remappings=[
            ('link_quality', f'/drone_{drone_id}/link_quality'),
            ('scan_control', f'/drone_{drone_id}/scan_control')  # 接收遠程掃描控制
        ],
        parameters=[{
            'use_sim_time': False,
        }]
    )
    
    # 2. velocity_control（速度控制）
    velocity_control_node = Node(
        package='ros2_px4_offboard_example',
        executable='velocity_control.py',
        name=f'velocity_control_drone_{drone_id}',
        output='screen',
        emulate_tty=True,
        remappings=[
            ('offboard_velocity_cmd', f'/drone_{drone_id}/offboard_velocity_cmd'),
            ('command', f'/drone_{drone_id}/command'),  # 接收鍵盤命令
            ('scan_control', f'/drone_{drone_id}/scan_control')  # 發布掃描控制
        ],
        parameters=[{
            'use_sim_time': False
        }]
    )
    
    # 注意：ROS2 Humble 沒有 ros2 topic relay
    # velocity_control 已經訂閱本地的 /fmu/... topics
    # 位置數據由 velocity_control 內部處理，不需要 relay
    
    return [
        fast_scan_node,
        velocity_control_node,
    ]


def generate_launch_description():
    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        default_value='1',
        description='无人机编号（1, 2, 3, ...）'
    )
    
    return LaunchDescription([
        drone_id_arg,
        OpaqueFunction(function=launch_setup)
    ])
