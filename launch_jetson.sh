#!/bin/bash
# Jetson 机载节点启动脚本
# 
# 用法：
#   ./launch_jetson.sh [无人机编号] [Tracker_A_ID] [Tracker_B_ID]
#   例如：./launch_jetson.sh 1 !aabbcc11 !aabbcc22

set -e

# 检查参数
if [ -z "$1" ]; then
    echo "错误: 请指定无人机编号"
    echo "用法: $0 [drone_id] [tracker_a_id] [tracker_b_id]"
    echo "例如: $0 1 !aabbcc11 !aabbcc22"
    echo ""
    echo "如果不指定 Tracker ID，将使用默认值（测试用）："
    echo "  Tracker A: !e2e5b7c4"
    echo "  Tracker B: !e2e5b8f8"
    exit 1
fi

DRONE_ID=$1
TRACKER_A_ID=${2:-"!e2e5b7c4"}  # 默认值
TRACKER_B_ID=${3:-"!e2e5b8f8"}  # 默认值

echo "=========================================="
echo "启动 Jetson 机载节点"
echo "无人机编号: $DRONE_ID"
echo "绑定 Tracker A: $TRACKER_A_ID"
echo "绑定 Tracker B: $TRACKER_B_ID"
echo "=========================================="

# Source ROS2 环境
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "错误: 未找到 ROS2 环境"
    exit 1
fi

echo ""
echo "启动节点..."
echo "  - fast_scan_node → /drone_${DRONE_ID}/link_quality"
echo "  - 扫描 Tracker: ${TRACKER_A_ID}, ${TRACKER_B_ID}"
echo "  - velocity_control → /drone_${DRONE_ID}/offboard_velocity_cmd"
echo ""

# 使用 launch 文件启动
ros2 launch px4_offboard jetson_onboard.launch.py \
    drone_id:=${DRONE_ID} \
    tracker_a_id:=${TRACKER_A_ID} \
    tracker_b_id:=${TRACKER_B_ID}
