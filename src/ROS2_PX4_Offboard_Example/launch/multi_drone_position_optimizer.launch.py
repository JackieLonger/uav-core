#!/usr/bin/env python3
"""
多无人机位置控制优化器启动脚本

用法：
    ros2 launch ros2_px4_offboard_example multi_drone_position_optimizer.launch.py drone_ids:='[1,2,3]'

配置：
    - num_drones: 无人机数量（默认3）
    - drone_ids: 无人机ID列表（默认[1, 2, 3]）
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'num_drones',
            default_value='3',
            description='无人机数量'
        ),

        DeclareLaunchArgument(
            'drone_ids',
            default_value='[1, 2, 3]',
            description='无人机ID列表'
        ),

        # 启动多无人机位置优化器
        Node(
            package='px4_offboard',
            executable='multi_drone_position_optimizer',
            name='multi_drone_position_optimizer',
            output='screen',
            parameters=[{
                'num_drones': LaunchConfiguration('num_drones'),
                'drone_ids': LaunchConfiguration('drone_ids'),
            }],
            emulate_tty=True,
        ),
    ])
