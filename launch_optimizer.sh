#!/bin/bash

# ================================================================================
# 🚀 UAV 信號優化系統 - 3 種快速啟動腳本
# ================================================================================
# 使用方式:
#   bash run_fast.sh        # 快速版 (3 次, 2-3 分鐘)
#   bash run_balanced.sh    # 平衡版 (5 次, 3-5 分鐘) [推薦]
#   bash run_precise.sh     # 精確版 (7 次, 5-7 分鐘)
# ================================================================================

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 函數: 印出標題
print_header() {
    echo -e "${BLUE}===============================================${NC}"
    echo -e "${BLUE}🚀 $1${NC}"
    echo -e "${BLUE}===============================================${NC}"
}

# 函數: 印出步驟
print_step() {
    echo -e "${GREEN}[✓] $1${NC}"
}

# 函數: 印出警告
print_warning() {
    echo -e "${YELLOW}[!] $1${NC}"
}

# 函數: 印出錯誤
print_error() {
    echo -e "${RED}[✗] $1${NC}"
}

# 函數: 檢查環境
check_environment() {
    print_header "環境檢查"
    
    # 檢查 ROS2
    if ! command -v ros2 &> /dev/null; then
        print_error "找不到 ros2 命令"
        print_warning "請先執行: source /opt/ros/humble/setup.bash"
        exit 1
    fi
    print_step "ROS2 環境正常"
    
    # 檢查專案目錄
    if [ ! -d "$HOME/uav-core" ]; then
        print_error "找不到 ~/uav-core 目錄"
        exit 1
    fi
    print_step "專案目錄存在"
    
    # 檢查編譯文件
    if [ ! -f "$HOME/uav-core/install/setup.bash" ]; then
        print_error "找不到編譯結果"
        print_warning "請先執行: cd ~/uav-core && colcon build"
        exit 1
    fi
    print_step "編譯文件存在"
    
    echo ""
}

# 函數: 進入目錄並準備環境
prepare_environment() {
    cd "$HOME/uav-core" || exit 1
    source install/setup.bash
    print_step "環境已載入"
}

# 函數: 快速版啟動 (3 次無改進)
run_fast_mode() {
    print_header "快速版啟動 (3 次無改進, 2-3 分鐘)"
    
    print_warning "確保已執行:"
    print_warning "  終端 1: MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600"
    echo ""
    
    prepare_environment
    
    echo -e "${YELLOW}即將啟動:${NC}"
    echo -e "  search_mode: fast"
    echo -e "  convergence_iterations: 3"
    echo -e "  預期時間: 2-3 分鐘"
    echo ""
    print_step "按 Enter 開始..."
    read -r
    
    ros2 launch px4_offboard single_drone_fast.launch.py \
        search_mode:=fast \
        convergence_iterations:=3
}

# 函數: 平衡版啟動 (5 次無改進) [推薦]
run_balanced_mode() {
    print_header "平衡版啟動 (5 次無改進, 3-5 分鐘) [推薦]"
    
    print_warning "確保已執行:"
    print_warning "  終端 1: MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600"
    echo ""
    
    prepare_environment
    
    echo -e "${YELLOW}即將啟動:${NC}"
    echo -e "  search_mode: fast"
    echo -e "  convergence_iterations: 5"
    echo -e "  預期時間: 3-5 分鐘 [推薦]"
    echo ""
    print_step "按 Enter 開始..."
    read -r
    
    ros2 launch px4_offboard single_drone_fast.launch.py \
        search_mode:=fast \
        convergence_iterations:=5
}

# 函數: 精確版啟動 (7 次無改進 + 完整搜索)
run_precise_mode() {
    print_header "精確版啟動 (7 次無改進 + 完整搜索, 5-7 分鐘)"
    
    print_warning "確保已執行:"
    print_warning "  終端 1: MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600"
    echo ""
    
    prepare_environment
    
    echo -e "${YELLOW}即將啟動:${NC}"
    echo -e "  search_mode: thorough"
    echo -e "  convergence_iterations: 7"
    echo -e "  預期時間: 5-7 分鐘"
    echo ""
    print_step "按 Enter 開始..."
    read -r
    
    ros2 launch px4_offboard single_drone_fast.launch.py \
        search_mode:=thorough \
        convergence_iterations:=7
}

# 函數: 自訂版啟動
run_custom_mode() {
    print_header "自訂版啟動"
    
    echo -e "${YELLOW}請輸入參數:${NC}"
    read -p "search_mode (fast/thorough) [fast]: " search_mode
    search_mode=${search_mode:-fast}
    
    read -p "convergence_iterations (1-10) [5]: " conv_iter
    conv_iter=${conv_iter:-5}
    
    read -p "drone_id [drone_1]: " drone_id
    drone_id=${drone_id:-drone_1}
    
    prepare_environment
    
    echo ""
    echo -e "${YELLOW}即將啟動:${NC}"
    echo -e "  search_mode: $search_mode"
    echo -e "  convergence_iterations: $conv_iter"
    echo -e "  drone_id: $drone_id"
    echo ""
    print_step "按 Enter 開始..."
    read -r
    
    ros2 launch px4_offboard single_drone_fast.launch.py \
        search_mode:="$search_mode" \
        convergence_iterations:="$conv_iter" \
        drone_id:="$drone_id"
}

# 主菜單
show_menu() {
    echo ""
    print_header "選擇啟動模式"
    echo -e "1) ${GREEN}⚡ 快速版${NC} (3 次, 2-3 分鐘) - 快速測試"
    echo -e "2) ${GREEN}⭐ 平衡版${NC} (5 次, 3-5 分鐘) - [推薦] 日常使用"
    echo -e "3) ${GREEN}🔬 精確版${NC} (7 次, 5-7 分鐘) - 精確搜索"
    echo -e "4) ${BLUE}🎛️  自訂版${NC} - 自訂參數"
    echo -e "5) ${YELLOW}❓ 幫助${NC} - 顯示使用說明"
    echo -e "6) ${RED}✗ 退出${NC}"
    echo ""
    read -p "選擇 (1-6): " choice
}

# 幫助頁面
show_help() {
    print_header "使用說明"
    cat << 'EOF'

📋 3 步快速開始:

1️⃣  終端 1 (Jetson) - 啟動通訊橋接:
    MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
    
    看到 "Connected!" 後繼續下一步

2️⃣  終端 2 (Jetson) - 啟動節點:
    bash run_balanced.sh
    
    或直接執行此腳本，選擇模式 2

3️⃣  終端 3 (筆電) - 可視化:
    export ROS_DOMAIN_ID=0 && export ROS_LOCALHOST_ONLY=0
    rviz2 -d ~/signal_optimization.rviz

📊 模式對比:

    快速版 (3)   : ⚡ 最快 (2-3 分鐘) - 系統測試
    平衡版 (5)   : ⭐ 推薦 (3-5 分鐘) - 日常使用 [推薦]
    精確版 (7)   : 🔬 最精 (5-7 分鐘) - 關鍵應用

🎯 預期成功標誌:

    [✓] 終端 1: "Connected!"
    [✓] 終端 2: "Signal optimizer v2 ready"
    [✓] 終端 2: "✅ New best signal" (多次)
    [✓] 終端 2: "🎯 收斂完成！切換到 HOLD 模式"
    [✓] RViz2: 無人機停止移動

❓ 有問題?

    查看文檔:
    - PARAMETER_QUICK_REFERENCE_ZH.md
    - QUICK_CONVERGENCE_CONFIG.md
    - COMMAND_EXECUTION_GUIDE.md

EOF
}

# 主程序
main() {
    check_environment
    
    while true; do
        show_menu
        
        case $choice in
            1)
                run_fast_mode
                break
                ;;
            2)
                run_balanced_mode
                break
                ;;
            3)
                run_precise_mode
                break
                ;;
            4)
                run_custom_mode
                break
                ;;
            5)
                show_help
                ;;
            6)
                print_step "退出"
                exit 0
                ;;
            *)
                print_error "無效選擇"
                ;;
        esac
    done
}

# 執行主程序
main
