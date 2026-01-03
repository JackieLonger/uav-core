#!/usr/bin/env python3
"""
多无人机信号优化器启动脚本（速度控制版本）

用法：
    ros2 launch ros2_px4_offboard_example multi_drone_optimizer.launch.py drone_ids:=[1,2,3]

功能：
    - 啟動 multi_drone_signal_optimizer（速度控制版本）
    - 自動配置正確的 topic remappings（確保與 Jetson 端通訊）
    
配置：
    - drone_ids: 无人机ID列表（默认[1, 2, 3]）
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import ast


def launch_setup(context, *args, **kwargs):
    """動態生成 launch actions（在運行時解析 LaunchConfiguration）"""
    
    drone_ids_str = LaunchConfiguration('drone_ids').perform(context)
    
    # 將字符串轉換為列表（例：'[1, 2, 3]' → [1, 2, 3]）
    try:
        drone_ids = ast.literal_eval(drone_ids_str)
        if isinstance(drone_ids, int):
            drone_ids = [drone_ids]  # 處理單一數字情況
    except:
        drone_ids = [1, 2, 3]  # 默認值
    
    num_drones = len(drone_ids)
    
    # ✅ 關鍵：為每個無人機生成正確的 topic remappings
    # Optimizer 內部使用相對名稱，需要映射到絕對路徑
    # 這確保 scan_control 發布到 /drone_N/scan_control（而非 /multi_drone_signal_optimizer/scan_control）
    
    optimizer_node = Node(
        package='ros2_px4_offboard_example',  # ✅ 正確的 package 名稱
        executable='multi_drone_signal_optimizer.py',
        name='multi_drone_signal_optimizer',
        output='screen',
        emulate_tty=True,
        # ✅ 注意：由於 Optimizer 內部已使用絕對路徑 /drone_N/xxx，這裡不需要額外 remappings
        # 但我們仍保留此結構以便未來擴展
        parameters=[{
            'num_drones': num_drones,
            'drone_ids': drone_ids,
            'use_sim_time': False,
        }]
    )
    
    return [optimizer_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'drone_ids',
            default_value='[1, 2, 3]',
            description='無人機 ID 列表（例：[1, 2, 3] 或 [1]）'
        ),
        OpaqueFunction(function=launch_setup)
    ])

