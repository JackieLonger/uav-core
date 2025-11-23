#!/bin/bash

################################################################################
# Jetson 端 - 單機無人機信號掃描啟動腳本（簡化版）
# 功能：啟動信號掃描節點 + 速度控制節點
# 使用方式：./launch_jetson_simple.sh <drone_id>
# 範例：./launch_jetson_simple.sh 1
################################################################################

if [ -z "$1" ]; then
    echo "錯誤：請提供無人機 ID"
    echo "使用方式：$0 <drone_id>"
    echo "範例：$0 1"
    exit 1
fi

DRONE_ID=$1

echo "========================================"
echo "Jetson 端 - Drone ${DRONE_ID} 系統啟動"
echo "========================================"

# 檢查是否在正確的目錄
if [ ! -f "install/setup.bash" ]; then
    echo "錯誤：請在 uav-core 根目錄執行此腳本"
    exit 1
fi

# Source ROS2 環境
echo "載入 ROS2 環境..."
source install/setup.bash

echo ""
echo "提醒：請確保已在 fast_scan_node.py 中設定正確的 Tracker ID！"
echo "檔案位置：src/ROS2_PX4_Offboard_Example/px4_offboard/fast_scan_node.py"
echo "需要修改：TRACKER_A_ID 和 TRACKER_B_ID"
echo ""
read -p "按 Enter 繼續..."

# 啟動 launch 文件（包含 fast_scan + velocity_control）
echo ""
echo "啟動 Drone ${DRONE_ID} 系統..."
echo "- fast_scan_node: 掃描 Meshtastic Tracker"
echo "- velocity_control: 接收速度命令並控制 PX4"
echo ""

ros2 launch ros2_px4_offboard_example jetson_onboard.launch.py drone_id:=${DRONE_ID}

echo ""
echo "========================================"
echo "Jetson 端系統已關閉"
echo "========================================"
