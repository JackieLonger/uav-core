#!/bin/bash
# 完整檢查清單 - 複製這個到終端執行

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🔍 無人機信號優化系統 - 部署檢查清單"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

WS="$HOME/uav-core"
cd "$WS" || exit 1

echo ""
echo "📋 [1/6] 檢查文件結構..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

files_to_check=(
    "src/ROS2_PX4_Offboard_Example/fast_scan_node.py"
    "src/ROS2_PX4_Offboard_Example/signal_optimizer_node.py"
    "src/ROS2_PX4_Offboard_Example/signal_visualizer_node.py"
    "src/ROS2_PX4_Offboard_Example/launch/single_drone.launch.py"
    "src/ROS2_PX4_Offboard_Example/config/signal_optimization.rviz"
    "config/default.yaml"
    "scripts/config_loader.py"
    "deploy/setup_jetson.sh"
)

for file in "${files_to_check[@]}"; do
    if [ -f "$file" ]; then
        echo "✅ $file"
    else
        echo "❌ $file (缺失)"
    fi
done

echo ""
echo "📋 [2/6] 檢查 .gitignore 配置..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if grep -q "^\*\.md" "$WS/.gitignore"; then
    echo "✅ .gitignore 已配置忽略 *.md"
else
    echo "⚠️  .gitignore 未配置，執行："
    echo "   echo '*.md' >> .gitignore"
fi

echo ""
echo "📋 [3/6] 檢查 Python 語法..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

python3 -m py_compile src/ROS2_PX4_Offboard_Example/signal_optimizer_node.py && \
    echo "✅ signal_optimizer_node.py 語法正確" || \
    echo "❌ signal_optimizer_node.py 有語法錯誤"

python3 -m py_compile src/ROS2_PX4_Offboard_Example/fast_scan_node.py && \
    echo "✅ fast_scan_node.py 語法正確" || \
    echo "❌ fast_scan_node.py 有語法錯誤"

echo ""
echo "📋 [4/6] 建議的 Git 提交..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "執行以下命令："
echo ""
echo "  git status"
echo "  git add -A"
echo "  git commit -m 'Fix signal_optimizer imports, add gitignore for md files'"
echo "  git push"
echo ""

echo "📋 [5/6] Jetson 端啟動檢查清單"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "在 Jetson 上執行 (3 個終端):"
echo ""
echo "終端 1 - Micro XRCE-DDS:"
echo "  $ MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600"
echo ""
echo "終端 2 - ROS2 啟動:"
echo "  $ source ~/uav-core/install/setup.bash"
echo "  $ ros2 launch ROS2_PX4_Offboard_Example single_drone.launch.py"
echo ""
echo "終端 3 - 監控 (可選):"
echo "  $ source ~/uav-core/install/setup.bash"
echo "  $ ros2 topic list"
echo "  $ ros2 topic echo /link_quality"
echo ""

echo "📋 [6/6] 筆電端啟動檢查清單 (另一個 Chat)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "待執行的筆電端任務 (使用 LAPTOP_SETUP.md):"
echo ""
echo "1. ✅ 安裝 ROS2 Humble + RViz2"
echo "2. ✅ 配置 CycloneDDS 遠端通訊"
echo "3. ✅ 啟動 RViz2"
echo "4. ✅ 驗證 topics 接收"
echo "5. ✅ RC 介入安全測試"
echo ""

echo "🚀 各文件功用速查表 (查看 FILE_REFERENCE.md)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "  QUICK_START.md         ← 快速啟動指南 + RViz2 設定"
echo "  LAPTOP_SETUP.md        ← 筆電端工作清單"
echo "  FILE_REFERENCE.md      ← 所有文件功用速查表 (本文件)"
echo "  SYSTEM_GUIDE.md        ← 完整系統文檔"
echo "  TESTING_PLAN.md        ← 測試檢查清單"
echo "  MIGRATION_GUIDE.md     ← 多機擴展指南"
echo ""

echo "✅ 檢查完成！"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
