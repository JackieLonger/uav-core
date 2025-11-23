#!/bin/bash

################################################################################
# 筆電端 - 多無人機信號優化系統啟動腳本
# 功能：啟動優化器和視覺化界面
# 使用方式：./launch_laptop.sh
################################################################################

echo "========================================"
echo "筆電端 - 多無人機信號優化系統"
echo "========================================"

# 檢查是否在正確的目錄
if [ ! -f "install/setup.bash" ]; then
    echo "錯誤：請在 uav-core 根目錄執行此腳本"
    exit 1
fi

# Source ROS2 環境
echo "載入 ROS2 環境..."
source install/setup.bash

# 選擇運行模式
echo ""
echo "請選擇運行模式："
echo "1) 啟動優化器 (multi_drone_signal_optimizer)"
echo "2) 啟動視覺化 (multi_drone_visualizer + RViz2)"
echo "3) 同時啟動優化器和視覺化"
echo "4) 僅啟動 RViz2"
echo ""
read -p "請輸入選項 [1-4]: " choice

case $choice in
    1)
        echo ""
        echo "啟動多無人機信號優化器..."
        echo "訂閱話題: /drone_1/link_quality, /drone_2/link_quality, /drone_3/link_quality"
        echo "發布話題: /drone_1/offboard_velocity_cmd, /drone_2/offboard_velocity_cmd, /drone_3/offboard_velocity_cmd"
        echo ""
        ros2 run ros2_px4_offboard_example multi_drone_signal_optimizer.py
        ;;
    2)
        echo ""
        echo "啟動視覺化系統..."
        echo ""
        # 在背景啟動 visualizer
        ros2 run ros2_px4_offboard_example multi_drone_visualizer.py &
        VISUALIZER_PID=$!
        
        sleep 2
        
        # 啟動 RViz2
        echo "啟動 RViz2..."
        rviz2 -d src/ROS2_PX4_Offboard_Example/config/multi_drone.rviz
        
        # 當 RViz2 關閉時，也關閉 visualizer
        kill $VISUALIZER_PID 2>/dev/null
        ;;
    3)
        echo ""
        echo "同時啟動優化器和視覺化..."
        echo ""
        
        # 啟動優化器（背景）
        ros2 run ros2_px4_offboard_example multi_drone_signal_optimizer.py &
        OPTIMIZER_PID=$!
        
        sleep 2
        
        # 啟動 visualizer（背景）
        ros2 run ros2_px4_offboard_example multi_drone_visualizer.py &
        VISUALIZER_PID=$!
        
        sleep 2
        
        # 啟動 RViz2（前景）
        rviz2 -d src/ROS2_PX4_Offboard_Example/config/multi_drone.rviz
        
        # 當 RViz2 關閉時，清理所有進程
        echo "關閉所有進程..."
        kill $OPTIMIZER_PID 2>/dev/null
        kill $VISUALIZER_PID 2>/dev/null
        ;;
    4)
        echo ""
        echo "啟動 RViz2..."
        rviz2 -d src/ROS2_PX4_Offboard_Example/config/multi_drone.rviz
        ;;
    *)
        echo "無效的選項"
        exit 1
        ;;
esac

echo ""
echo "========================================"
echo "筆電端系統已關閉"
echo "========================================"
