#!/bin/bash
# RViz2 可视化启动脚本

set -e

cd "$(dirname "$0")"
source install/setup.bash

echo "=========================================="
echo "启动 RViz2 多无人机可视化"
echo "=========================================="
echo "重要：每架无人机绑定不同的两个 Tracker"
echo "请确保 Tracker ID 与 Jetson 启动时一致"
echo "=========================================="

# Tracker 绑定配置
# 格式：{无人机ID: [TrackerA_ID, TrackerB_ID]}
TRACKER_BINDINGS="{
    1: ['!tracker1A', '!tracker1B'],
    2: ['!tracker2A', '!tracker2B'],
    3: ['!tracker3A', '!tracker3B']
}"

echo "当前 Tracker 绑定："
echo "  Drone 1 → !tracker1A & !tracker1B"
echo "  Drone 2 → !tracker2A & !tracker2B"
echo "  Drone 3 → !tracker3A & !tracker3B"
echo "=========================================="

# 启动可视化节点
ros2 run px4_offboard multi_drone_visualizer \
    --ros-args \
    -p num_drones:=3 \
    -p drone_ids:="[1, 2, 3]" \
    -p drone_tracker_bindings:="$TRACKER_BINDINGS" &

VISUALIZER_PID=$!

# 等待1秒让可视化节点启动
sleep 1

# 启动 RViz2
rviz2 -d src/ROS2_PX4_Offboard_Example/resource/multi_drone.rviz &

RVIZ_PID=$!

echo ""
echo "✓ 可视化节点已启动 (PID: $VISUALIZER_PID)"
echo "✓ RViz2 已启动 (PID: $RVIZ_PID)"
echo ""
echo "按 Ctrl+C 停止..."

# 等待中断信号
trap "kill $VISUALIZER_PID $RVIZ_PID 2>/dev/null" EXIT

wait
