#!/usr/bin/env python3
"""
Single Drone Fast Test Launch (v2)
快速 10 分鐘測試啟動文件

使用方式:
# 快速測試 (3 次無改進停留)
ros2 launch px4_offboard single_drone_fast.launch.py search_mode:=fast convergence_iterations:=3

# 徹底搜索 (7 次位置測試)
ros2 launch px4_offboard single_drone_fast.launch.py search_mode:=thorough convergence_iterations:=7 drone_id:=drone_1

參數:
  search_mode: 'fast' (2×2m, 快速) 或 'thorough' (3×3m, 徹底)
  drone_id: 無人機編號 (預設: drone_1)
  convergence_iterations: 連續N次無改進則停留 (預設: 5)
    - 3: 快速模式 (約 2-3 分鐘完成)
    - 5: 平衡模式 (約 3-5 分鐘完成) [預設]
    - 7: 徹底模式 (約 5-7 分鐘完成)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ===== 宣告參數 =====
    search_mode_arg = DeclareLaunchArgument(
        'search_mode',
        default_value='fast',
        description='Search mode: fast (2x2m) or thorough (3x3m)'
    )
    
    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        default_value='drone_1',
        description='Drone ID for multi-drone support'
    )
    
    convergence_iterations_arg = DeclareLaunchArgument(
        'convergence_iterations',
        default_value='5',
        description='Number of iterations with no improvement before holding position'
    )
    
    # ===== 獲取參數 =====
    search_mode = LaunchConfiguration('search_mode')
    drone_id = LaunchConfiguration('drone_id')
    convergence_iterations = LaunchConfiguration('convergence_iterations')
    
    # ===== 節點 1: fast_scan_node (Meshtastic 掃描) =====
    fast_scan_node = Node(
        package='px4_offboard',
        executable='fast_scan',
        name='fast_scan_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'probe_interval': 5.0},
            {'response_timeout': 30.0}
        ]
    )
    
    # ===== 節點 2: signal_optimizer_node_v2 (位置優化 + 停留) =====
    signal_optimizer_node = Node(
        package='px4_offboard',
        executable='signal_optimizer_v2',
        name='signal_optimizer_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'search_mode': search_mode},
            {'drone_id': drone_id},
            {'convergence_iterations': convergence_iterations},
            {'signal_smooth_window': 3}
        ]
    )
    
    # ===== 節點 3: signal_visualizer_node_v2 (RViz2 可視化) =====
    signal_visualizer_node = Node(
        package='px4_offboard',
        executable='signal_visualizer_v2',
        name='signal_visualizer_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'drone_id': drone_id}
        ]
    )
    
    # ===== 組合啟動描述 =====
    ld = LaunchDescription([
        search_mode_arg,
        drone_id_arg,
        convergence_iterations_arg,
        fast_scan_node,
        signal_optimizer_node,
        signal_visualizer_node,
    ])
    
    return ld
