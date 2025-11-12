#!/bin/bash
# 多无人机信号优化器启动脚本
# 
# 用法：
#   ./launch_multi_drone_optimizer.sh [无人机数量]
#   例如：./launch_multi_drone_optimizer.sh 3

set -e

# 进入工作目录
cd "$(dirname "$0")"

# Source ROS2 环境
source install/setup.bash

# 参数
NUM_DRONES=${1:-3}
echo "=========================================="
echo "启动多无人机信号优化器"
echo "无人机数量: $NUM_DRONES"
echo "=========================================="

# 检查依赖
if ! command -v ros2 &> /dev/null; then
    echo "错误: 未找到 ROS2 环境"
    exit 1
fi

# 启动节点
ros2 run px4_offboard multi_drone_signal_optimizer \
    --ros-args \
    -p num_drones:=$NUM_DRONES \
    -p drone_ids:="[1, 2, 3]"
