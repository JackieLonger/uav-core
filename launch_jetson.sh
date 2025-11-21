#!/bin/bash
# Jetson 机载节点启动脚本
# 
# 用法：
#   ./launch_jetson.sh [无人机编号]
#   例如：./launch_jetson.sh 1
#
# 注意：Tracker ID 已在 fast_scan_node.py 中硬编码，
#       每架无人机的 Jetson 需手动修改该文件中的配置

set -e

# 检查参数
if [ -z "$1" ]; then
    echo "错误: 请指定无人机编号"
    echo "用法: $0 [drone_id]"
    echo "例如: $0 1"
    echo ""
    echo "注意: Tracker ID 已在 fast_scan_node.py 中硬编码"
    echo "      每架 Jetson 需手动修改该文件的配置区域"
    exit 1
fi

DRONE_ID=$1

echo "=========================================="
echo "启动 Jetson 机载节点"
echo "无人机编号: $DRONE_ID"
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
echo "  - velocity_control → /drone_${DRONE_ID}/offboard_velocity_cmd"
echo ""
echo "注意: Tracker ID 已在 fast_scan_node.py 中硬编码"
echo ""

# 使用 launch 文件启动
ros2 launch px4_offboard jetson_onboard.launch.py \
    drone_id:=${DRONE_ID}
