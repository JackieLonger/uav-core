#!/usr/bin/env python3
"""
Jetson 机载节点启动文件

用法（在每架无人机的 Jetson 上运行）：
    ros2 launch px4_offboard jetson_onboard.launch.py drone_id:=1
    ros2 launch px4_offboard jetson_onboard.launch.py drone_id:=2
    ros2 launch px4_offboard jetson_onboard.launch.py drone_id:=3

功能：
- 启动 fast_scan_node（扫描 Meshtastic）
- 启动 velocity_control（接收速度指令 → 发给 Pixhawk）
- 自动配置正确的 topic 名称

注意：
- Tracker ID 已在 fast_scan_node.py 中硬编码
- 每架无人机的 Jetson 需手动修改 fast_scan_node.py 的配置区域
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # 声明参数
    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        default_value='1',
        description='无人机编号（1, 2, 3, ...）'
    )
    
    drone_id = LaunchConfiguration('drone_id')
    
    # 1. fast_scan_node（扫描 Meshtastic）
    # 注意：Tracker ID 已在节点代码中硬编码
    fast_scan_node = Node(
        package='px4_offboard',
        executable='fast_scan',
        name=['fast_scan_drone_', drone_id],
        output='screen',
        emulate_tty=True,
        remappings=[
            ('link_quality', ['/drone_', drone_id, '/link_quality'])
        ],
        parameters=[{
            'use_sim_time': False,
        }]
    )
    
    # 2. velocity_control（速度控制）
    velocity_control_node = Node(
        package='px4_offboard',
        executable='velocity_control',
        name=['velocity_control_drone_', drone_id],
        output='screen',
        emulate_tty=True,
        remappings=[
            ('offboard_velocity_cmd', ['/drone_', drone_id, '/offboard_velocity_cmd'])
        ],
        parameters=[{
            'use_sim_time': False
        }]
    )
    
    return LaunchDescription([
        drone_id_arg,
        fast_scan_node,
        velocity_control_node,
    ])
