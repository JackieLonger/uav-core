#!/bin/bash
# filepath: /home/jackiehp/uav-core/launch_laptop.sh

################################################################################
# 筆電端 - 多無人機信號優化系統啟動腳本
# 修改版：選項3 改為背景執行 RViz，無需 tmux 切換
################################################################################

echo "========================================"
echo "筆電端 - 多無人機信號優化系統"
echo "========================================"

# 檢查是否安裝 tmux (雖然選項3不用了，但保留檢查以免其他選項需要)
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
echo "1) 啟動優化器 (multi_drone_signal_optimizer)"
echo "2) 啟動視覺化 (multi_drone_visualizer + RViz2)"
echo "3) 同時啟動優化器和視覺化 (背景執行 RViz，無需切換視窗)"
echo "4) 僅啟動 RViz2"
echo ""
read -p "請輸入選項 [1-4]: " choice

# 定義 RViz 啟動函數 (避免重複代碼)
launch_rviz_bg() {
    echo "正在背景啟動 RViz2..."
    source install/setup.bash
    # 清除可能導致衝突的環境變數 (參考原腳本)
    unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE GIO_MODULE_DIR LOCPATH
    # 啟動 RViz 並將輸出丟入黑洞，避免干擾終端機
    /opt/ros/humble/bin/rviz2 -d src/ROS2_PX4_Offboard_Example/resource/multi_drone.rviz > /dev/null 2>&1 &
    # 回傳 PID
    echo $!
}

case $choice in
    1)
        echo ""
        echo "啟動多無人機信號優化器..."
        read -p "請輸入要控制的無人機 ID (逗號分隔，例: 1,2,3 或 1): " drone_input
        
        # 轉換輸入為陣列
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
        echo "啟動 Drone 1 系統..."
        echo "無人機 ID: $DRONE_IDS"
        echo ""
        source install/setup.bash
        ros2 run ros2_px4_offboard_example multi_drone_signal_optimizer.py --ros-args -p drone_ids:="$DRONE_IDS"
        ;;
        
    2)
        # ⚠️ 注意：截圖中未顯示此區塊代碼。
        # 如果你原本這裡有執行特定的 visualizer python 檔，請將其補回。
        # 目前僅設定為啟動 RViz。
        echo ""
        echo "啟動視覺化..."
        source install/setup.bash
        unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE GIO_MODULE_DIR LOCPATH
        /opt/ros/humble/bin/rviz2 -d src/ROS2_PX4_Offboard_Example/resource/multi_drone.rviz
        ;;

    3)
        # --- 這是修改後的核心部分 ---
        echo ""
        echo "同時啟動優化器和視覺化..."
        read -p "請輸入要控制的無人機 ID (逗號分隔，例: 1,2,3 或 1): " drone_input
        
        # 轉換輸入為陣列
        IFS=',' read -ra DRONE_ARRAY <<< "$drone_input"
        DRONE_IDS="["
        for i in "${!DRONE_ARRAY[@]}"; do
            DRONE_IDS="${DRONE_IDS}${DRONE_ARRAY[$i]}"
            if [ $i -lt $((${#DRONE_ARRAY[@]} - 1)) ]; then
                DRONE_IDS="${DRONE_IDS}, "
            fi
        done
        DRONE_IDS="${DRONE_IDS}]"
        
        # 1. 在背景啟動 RViz
        RVIZ_PID=$(launch_rviz_bg)
        
        # 設定捕捉訊號：當腳本結束或被 Ctrl+C 時，自動關閉 RViz
        trap "echo '正在關閉 RViz...'; kill $RVIZ_PID" EXIT
        
        echo "RViz 已在背景執行 (PID: $RVIZ_PID)"
        echo "啟動優化器 (請在此直接輸入指令，如 1, 2, Q)..."
        echo "-----------------------------------------------------"
        
        # 2. 在前景執行優化器 (這是你會操作的部分)
        source install/setup.bash
        ros2 run ros2_px4_offboard_example multi_drone_signal_optimizer.py --ros-args -p drone_ids:="$DRONE_IDS"
        
        # 程式結束後，trap 會自動觸發並關閉 RViz
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