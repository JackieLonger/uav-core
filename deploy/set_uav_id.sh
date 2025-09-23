#!/usr/bin/env bash
set -euo pipefail
ID="${1:-}"
if [ -z "$ID" ]; then echo "用法: $0 uav-001"; exit 1; fi
sudo mkdir -p /etc/uav
echo "UAV_ID=$ID" | sudo tee /etc/uav/id >/dev/null
echo "[OK] 已設定 UAV_ID=$ID 到 /etc/uav/id"
echo "提示：systemd 可用 EnvironmentFile=/etc/uav/id；或在 shell 用：export \$(cat /etc/uav/id)"
