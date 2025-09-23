#!/usr/bin/env bash
set -euo pipefail
WS=${WS:-$HOME/uav_core_ws}
sudo apt update
sudo apt install -y python3-vcstool python3-colcon-common-extensions build-essential cmake ninja-build

mkdir -p "$WS/src"
cd "$WS"
# 匯入外部依賴
vcs import src < "$HOME/uav-core/thirdparty.repos" || true
# 放入自家套件（用 rsync 複製）
mkdir -p src/_uav_core
rsync -a --delete "$HOME/uav-core/src/" "src/_uav_core/"

colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
grep -q "uav_core_ws" ~/.bashrc || echo "source $WS/install/setup.bash" >> ~/.bashrc
echo "[OK] setup_jetson done."
