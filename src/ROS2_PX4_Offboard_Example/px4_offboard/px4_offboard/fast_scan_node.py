#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
import meshtastic
import meshtastic.serial_interface
from pubsub import pub

import time
import threading
import csv
import os
from datetime import datetime
from meshtastic.protobuf import mesh_pb2, portnums_pb2
import json

# ========== LoRa 參數設置 ==========
# Meshtastic 默認使用 LoRa SF11 BW125kHz
LORA_BW_HZ = 125000            # LoRa 頻寬 (Hz)
LORA_NF_DB = 6.0               # 預估射頻前端噪聲指數 (dB)
THERMAL_NOISE_FLOOR = -174     # 熱噪聲底 (dBm/Hz)

def calculate_noise_floor(bw_hz=LORA_BW_HZ, nf_db=LORA_NF_DB):
    """
    根據熱噪聲模型計算接收機噪聲底噪
    
    公式: N = -174 dBm/Hz + 10*log10(BW_Hz) + NF
    
    Args:
        bw_hz: LoRa 頻寬 (Hz)
        nf_db: 噪聲指數 (dB)
    
    Returns:
        噪聲底噪 (dBm)
    """
    import math
    bw_term = 10 * math.log10(bw_hz)
    noise_floor = THERMAL_NOISE_FLOOR + bw_term + nf_db
    return noise_floor

# 預計算噪聲底噪
NOISE_FLOOR_DBM = calculate_noise_floor()

# ========== 掃描設定 ==========
TARGET_NODE_IDS = [
    #"!e2e5b7c4",
    "!e2e5b980 "
]

PROBE_INTERVAL = 5          # 每次掃描週期（秒）
DELAY_BETWEEN_NODES = 10     # 節點間等待時間（秒）
RESPONSE_TIMEOUT = 30       # 等待回應秒數
CSV_FILENAME = "fast_scan_log.csv"


class FastScanNode(Node):
    def __init__(self):
        super().__init__('fast_scan_node')

        # === Callback Group 確保 thread 安全 ===
        self.callback_group = ReentrantCallbackGroup()

        # === ROS Publisher ===
        self.link_pub = self.create_publisher(String, 'link_quality', 10)

        # === 狀態變數 ===
        self.response_event = threading.Event()
        self.results = {}
        self.current_probing_target = None
        self.interface = None
        self.scanning = False

        # === CSV 初始檔案 ===
        self.setup_csv_file()

        # === Meshtastic 監聽 ===
        pub.subscribe(self.onReceive, "meshtastic.receive")

        # === 啟動 ROS2 timer ===
        self.timer = self.create_timer(PROBE_INTERVAL + 2, self.timer_callback, callback_group=self.callback_group)

        self.get_logger().info(f"ROS2 FastScanNode 啟動，將輪詢 {len(TARGET_NODE_IDS)} 個節點。")

    # ---------------------------------
    def setup_csv_file(self):
        file_exists = os.path.isfile(CSV_FILENAME)
        with open(CSV_FILENAME, mode='a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow([
                    "Timestamp",
                    "TargetID",
                    "Status",
                    "Forward_RSSI",
                    "Forward_SNR",
                    "Return_RSSI",
                    "Return_SNR"
                ])
        self.get_logger().info(f"日誌檔 '{CSV_FILENAME}' 準備就緒。")

    # ---------------------------------
    def onReceive(self, packet, interface):
        port = packet.get('decoded', {}).get('portnum')
        from_id = packet.get('fromId', 'unknown')
        self.get_logger().debug(f"接收到封包: from={from_id}, port={port}")

        is_traceroute_response = (
            port == "TRACEROUTE_APP" or port == portnums_pb2.PortNum.TRACEROUTE_APP
        )

        if is_traceroute_response and from_id == self.current_probing_target:
            rd = mesh_pb2.RouteDiscovery()
            try:
                rd.ParseFromString(packet['decoded']['payload'])
            except Exception:
                rd = None

            UNK_SNR = -128
            self.results['status'] = 'Success'
            
            # 回程訊號（直接測量）
            return_rssi = packet.get('rxRssi', 0)
            return_snr = packet.get('rxSnr', 0)
            self.results['return_rssi'] = return_rssi
            self.results['return_snr'] = return_snr
            
            self.get_logger().debug(f"回程 RSSI={return_rssi} dBm, SNR={return_snr} dB")

            if rd:
                # 去程 SNR (從路由發現訊息獲得)
                forward_snr = 'N/A'
                if hasattr(rd, "snr_towards") and len(rd.snr_towards) > 0 and rd.snr_towards[-1] != UNK_SNR:
                    forward_snr = rd.snr_towards[-1] / 4.0
                    self.results['forward_snr'] = forward_snr
                else:
                    self.results['forward_snr'] = 'N/A'

                # ============ 去程 RSSI 推算 ============
                # 使用公式 (1)：RSSI_forward = SNR_forward + N_B
                # 其中 N_B 是對端接收機的噪聲底噪
                
                forward_rssi = 'N/A'
                
                if isinstance(forward_snr, (int, float)) and forward_snr != 'N/A':
                    # 方法 1: 精確公式 (需知道對端噪聲底噪)
                    # 假設對端與本端使用相同的 LoRa 參數
                    forward_rssi_exact = forward_snr + NOISE_FLOOR_DBM
                    
                    # 方法 2: 近似公式 (不需知道噪聲底噪，但假設兩端條件相近)
                    # RSSI_forward ≈ RSSI_return + (SNR_forward - SNR_return)
                    forward_rssi_approx = return_rssi + (forward_snr - return_snr)
                    
                    # 取近似公式的結果（因為實務上兩端條件通常相近）
                    forward_rssi = round(forward_rssi_approx, 1)
                    self.results['forward_rssi'] = forward_rssi
                    
                    self.get_logger().debug(
                        f"去程 RSSI 推算: "
                        f"精確式={forward_rssi_exact:.1f} dBm, "
                        f"近似式={forward_rssi_approx:.1f} dBm, "
                        f"噪聲底噪={NOISE_FLOOR_DBM:.1f} dBm"
                    )
                else:
                    self.results['forward_rssi'] = 'N/A'

                # 回程 SNR 參考值 (從路由發現訊息獲得)
                if hasattr(rd, "snr_back") and len(rd.snr_back) > 0 and rd.snr_back[-1] != UNK_SNR:
                    return_snr_ref = rd.snr_back[-1] / 4.0
                    self.results['return_snr_ref'] = return_snr_ref
            else:
                self.results['forward_snr'] = 'N/A'
                self.results['forward_rssi'] = 'N/A'

            self.response_event.set()

    # ---------------------------------
    def probe_node(self, target_id):
        """對單一節點發送探測"""
        self.current_probing_target = target_id
        self.results = {'status': 'Timeout'}
        self.response_event.clear()

        self.get_logger().info(f"發送探測請求至 {target_id}...")

        try:
            self.interface.sendData(
                mesh_pb2.RouteDiscovery(),
                destinationId=target_id,
                portNum=portnums_pb2.PortNum.TRACEROUTE_APP,
                wantResponse=True
            )
        except Exception as e:
            self.get_logger().error(f"發送失敗: {e}")
            return

        # 等待回應
        self.response_event.wait(timeout=RESPONSE_TIMEOUT)

        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        status = self.results['status']
        self.get_logger().info(f"探測狀態: {status}")

        # === 寫入 CSV ===
        with open(CSV_FILENAME, mode='a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([
                timestamp,
                target_id,
                status,
                self.results.get('forward_rssi', ''),
                self.results.get('forward_snr', ''),
                self.results.get('return_rssi', ''),
                self.results.get('return_snr', '')
            ])

        # === 發布 ROS2 topic ===
        msg_data = {
            "timestamp": timestamp,
            "target_id": target_id,
            "status": status,
            "forward_rssi": self.results.get('forward_rssi', ''),
            "forward_snr": self.results.get('forward_snr', ''),
            "return_rssi": self.results.get('return_rssi', ''),
            "return_snr": self.results.get('return_snr', '')
        }
        ros_msg = String()
        ros_msg.data = json.dumps(msg_data, ensure_ascii=False)
        self.link_pub.publish(ros_msg)
        self.get_logger().info(f"已發布 link_quality: {ros_msg.data}")

        # === 重要：延遲並清理 ===
        time.sleep(1.5)
        try:
            if hasattr(self.interface, "flush"):
                self.interface.flush()
        except Exception:
            pass

    # ---------------------------------
    def timer_callback(self):
        if self.scanning:
            return
        self.scanning = True

        if not self.interface:
            try:
                self.interface = meshtastic.serial_interface.SerialInterface()
                self.get_logger().info(f"Meshtastic serial interface 啟動成功。")
            except Exception as e:
                self.get_logger().error(f"無法開啟 Meshtastic serial interface: {e}")
                self.scanning = False
                return

        self.get_logger().info(f"開始一輪快速掃描...")

        for i, target_id in enumerate(TARGET_NODE_IDS):
            self.probe_node(target_id)
            if i < len(TARGET_NODE_IDS) - 1:
                self.get_logger().info(f"等待 {DELAY_BETWEEN_NODES} 秒後探測下一個節點...")
                time.sleep(DELAY_BETWEEN_NODES)

        self.get_logger().info(f"本輪掃描完成，等待下次掃描...")
        self.scanning = False

    # ---------------------------------
    def destroy_node(self):
        self.get_logger().info("FastScanNode 結束運行，關閉連線...")
        if self.interface:
            try:
                self.interface.close()
            except Exception:
                pass
        super().destroy_node()


# ---------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = FastScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("程式被使用者中斷，正在關閉...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
