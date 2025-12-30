#!/bin/bash

###########################################################
# Jetson 端 - 單機無人機「位置控制」啟動腳本（簡化版）
# 功能：啟動 position_control 節點 + fast_scan 節點（默認關閉掃描）
# 使用方式：./launch_jetson_position.sh <drone_id>
# 範例：./launch_jetson_position.sh 1
#
# 注意：掃描功能默認關閉，由 Laptop 按 F 鍵啟動
###########################################################

if [ -z "$1" ]; then
    echo "錯誤：請提供無人機 ID"
    echo "使用方式：$0 <drone_id>"
    echo "範例：$0 1"
    exit 1
fi

DRONE_ID=$1

echo "========================================"
echo "Jetson 端 - Drone ${DRONE_ID} 位置控制啟動"
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
echo "⚠️  掃描功能默認關閉，請在 Laptop 上按 F 鍵啟動掃描"
echo ""
read -p "按 Enter 繼續..."

# 啟動 launch 文件（包含 fast_scan + position_control）
echo ""
echo "啟動 Drone ${DRONE_ID} 系統（位置控制）..."
echo "- position_control: 接收座標命令並控制 PX4"
echo "- fast_scan_node: 等待 Laptop 按 F 鍵啟動掃描"
echo ""

ros2 launch ros2_px4_offboard_example jetson_onboard_position.launch.py drone_id:=${DRONE_ID}

echo ""
echo "========================================"
echo "Jetson 端系統已關閉"
echo "========================================"
