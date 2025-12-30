#!/bin/bash
# filepath: /home/jackiehp/uav-core/launch_laptop_position.sh

################################################################################
# 筆電端 - 多無人機「位置控制」信號優化系統啟動腳本
################################################################################

echo "========================================"
echo "筆電端 - 多無人機位置控制優化系統"
echo "========================================"

# 檢查是否安裝 tmux（本腳本不強制使用，但保留檢查）
if ! command -v tmux &> /dev/null; then
    echo "⚠️  未安裝 tmux，正在安裝..."
    sudo apt install -y tmux
fi

# 檢查是否在正確的目錄
if [ ! -f "install/setup.bash" ]; then
    echo "錯誤：請在 uav-core 根目錄執行此腳本"
    exit 1
fi

# 選擇運行模式
echo ""
echo "請選擇運行模式："
echo "1) 啟動位置優化器 (multi_drone_position_optimizer)"
echo "2) 啟動視覺化 (multi_drone_visualizer + RViz2)"
echo "3) 同時啟動優化器和視覺化 (背景執行 RViz，無需切換視窗)"
echo "4) 僅啟動 RViz2"
echo ""
read -p "請輸入選項 [1-4]: " choice

# 定義 RViz 啟動函數 (避免重複代碼)
launch_rviz_bg() {
    echo "正在背景啟動 RViz2..."
    source install/setup.bash
    unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE GIO_MODULE_DIR LOCPATH
    /opt/ros/humble/bin/rviz2 -d src/ROS2_PX4_Offboard_Example/resource/multi_drone.rviz > /dev/null 2>&1 &
    echo $!
}

case $choice in
    1)
        echo ""
        echo "啟動多無人機位置控制優化器..."
        read -p "請輸入要控制的無人機 ID (逗號分隔，例: 1,2,3 或 1): " drone_input
        IFS=',' read -ra DRONE_ARRAY <<< "$drone_input"
        DRONE_IDS="["
        for i in "${!DRONE_ARRAY[@]}"; do
            DRONE_IDS="${DRONE_IDS}${DRONE_ARRAY[$i]}"
            if [ $i -lt $((${#DRONE_ARRAY[@]} - 1)) ]; then
                DRONE_IDS="${DRONE_IDS}, "
            fi
        done
        DRONE_IDS="${DRONE_IDS}]"

        echo ""
        echo "無人機 ID: $DRONE_IDS"
        echo ""
        source install/setup.bash
        ros2 run ros2_px4_offboard_example multi_drone_position_optimizer --ros-args -p drone_ids:="${DRONE_IDS}"
        ;;

    2)
        echo ""
        echo "啟動視覺化..."
        source install/setup.bash
        unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE GIO_MODULE_DIR LOCPATH
        /opt/ros/humble/bin/rviz2 -d src/ROS2_PX4_Offboard_Example/resource/multi_drone.rviz
        ;;

    3)
        echo ""
        echo "同時啟動優化器和視覺化..."
        read -p "請輸入要控制的無人機 ID (逗號分隔，例: 1,2,3 或 1): " drone_input
        IFS=',' read -ra DRONE_ARRAY <<< "$drone_input"
        DRONE_IDS="["
        for i in "${!DRONE_ARRAY[@]}"; do
            DRONE_IDS="${DRONE_IDS}${DRONE_ARRAY[$i]}"
            if [ $i -lt $((${#DRONE_ARRAY[@]} - 1)) ]; then
                DRONE_IDS="${DRONE_IDS}, "
            fi
        done
        DRONE_IDS="${DRONE_IDS}]"

        RVIZ_PID=$(launch_rviz_bg)
        trap "echo '正在關閉 RViz...'; kill $RVIZ_PID" EXIT

        echo "RViz 已在背景執行 (PID: $RVIZ_PID)"
        echo "啟動優化器 (請在此直接輸入指令，如 1, 2, Q)..."
        echo "-----------------------------------------------------"

        source install/setup.bash
        ros2 run ros2_px4_offboard_example multi_drone_position_optimizer --ros-args -p drone_ids:="${DRONE_IDS}"
        ;;

    4)
        echo ""
        echo "僅啟動 RViz2..."
        source install/setup.bash
        unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE GIO_MODULE_DIR LOCPATH
        /opt/ros/humble/bin/rviz2 -d src/ROS2_PX4_Offboard_Example/resource/multi_drone.rviz
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
