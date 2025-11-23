#!/bin/bash
# 模擬 Jetson 發送訊號測試腳本
# 使用單引號避免 Bash 歷史擴展問題

set -e

cd ~/uav-core
source install/setup.bash

echo "====================================="
echo "開始模擬 Drone 1 發送訊號"
echo "====================================="
echo ""

COUNTER=1

while true; do
  echo "--- 循環 $COUNTER ---"
  
  echo "發送 Tracker A 訊號..."
  ros2 topic pub --once /drone_1/link_quality std_msgs/msg/String \
    'data: "{\"target_id\": \"!TEST_A\", \"status\": \"Success\", \"forward_rssi\": -80, \"forward_snr\": 10, \"return_rssi\": -85, \"return_snr\": 8, \"timestamp\": \"123\"}"'
  
  sleep 1
  
  echo "發送 Tracker B 訊號..."
  ros2 topic pub --once /drone_1/link_quality std_msgs/msg/String \
    'data: "{\"target_id\": \"!TEST_B\", \"status\": \"Success\", \"forward_rssi\": -75, \"forward_snr\": 12, \"return_rssi\": -78, \"return_snr\": 11, \"timestamp\": \"123\"}"'
  
  echo "等待 3 秒..."
  sleep 3
  
  COUNTER=$((COUNTER + 1))
done
