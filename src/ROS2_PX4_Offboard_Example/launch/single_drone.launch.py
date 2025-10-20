from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Meshtastic 掃描節點 - 掃描指定 Tracker 節點並獲取 SNR/RSSI
        Node(
            package='ros2_px4_offboard_example',
            executable='fast_scan_node.py',
            name='fast_scan',
            output='screen'
        ),
        
        # 位置優化節點 - 根據訊號品質計算最佳位置並控制無人機
        Node(
            package='ros2_px4_offboard_example',
            executable='signal_optimizer_node.py',
            name='signal_optimizer',
            output='screen'
        ),
        
        # 視覺化節點 - 在 RViz2 中顯示訊號熱力圖、軌跡等
        Node(
            package='ros2_px4_offboard_example',
            executable='signal_visualizer_node.py',
            name='signal_visualizer',
            output='screen'
        )
    ])