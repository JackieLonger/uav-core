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
from launch.actions import DeclareLaunchArgument, ExecuteProcess
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
        package='ros2_px4_offboard_example',
        executable='fast_scan_node.py',
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
        package='ros2_px4_offboard_example',
        executable='velocity_control.py',
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
    
    # 3. Topic relay - 将 PX4 数据重映射为带 drone_id 的版本
    # 使用 ExecuteProcess 运行 ros2 topic relay 命令
    relay_position = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'relay',
            '/fmu/out/vehicle_local_position',
            ['/drone_', drone_id, '/fmu/out/vehicle_local_position']
        ],
        output='screen',
        shell=False
    )
    
    relay_status = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'relay',
            '/fmu/out/vehicle_status',
            ['/drone_', drone_id, '/fmu/out/vehicle_status']
        ],
        output='screen',
        shell=False
    )
    
    return LaunchDescription([
        drone_id_arg,
        fast_scan_node,
        velocity_control_node,
        relay_position,
        relay_status,
    ])
