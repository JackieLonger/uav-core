#!/bin/bash
# 啟動優化器測試腳本

set -e

cd ~/uav-core
source install/setup.bash

echo "====================================="
echo "啟動多無人機信號優化器"
echo "模式：單機測試 (Drone 1)"
echo "====================================="
echo ""

ros2 run px4_offboard multi_drone_signal_optimizer \
  --ros-args \
  -p num_drones:=1 \
  -p drone_ids:="[1]"
