#!/usr/bin/env python3
# 使用 TEXT_MESSAGE 測試連通性（而不是 TRACEROUTE）

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
import meshtastic
import meshtastic.serial_interface
from pubsub import pub
import time, threading, csv, os
from datetime import datetime
from meshtastic.protobuf import mesh_pb2, portnums_pb2
import json, math

# LoRa 參數設置
LORA_BW_HZ = 125000
LORA_NF_DB = 6.0
THERMAL_NOISE_FLOOR = -174
NOISE_FLOOR_DBM = THERMAL_NOISE_FLOOR + 10 * math.log10(LORA_BW_HZ) + LORA_NF_DB

# 掃描設定 - TEXT_MESSAGE 版本測試
TARGET_NODE_IDS = ["!e2e5b7c4", "!e2e5b8f8"]
PROBE_INTERVAL = 8
DELAY_BETWEEN_NODES = 60
RESPONSE_TIMEOUT = 45
PRE_PROBE_DELAY = 8
POST_PROBE_DELAY = 3
CSV_FILENAME = "fast_scan_log_textmsg.csv"

class FastScanNode(Node):
    def __init__(self):
        super().__init__('fast_scan_node_textmsg')
        self.callback_group = ReentrantCallbackGroup()
        self.link_pub = self.create_publisher(String, 'link_quality_textmsg', 10)
        self.response_event = threading.Event()
        self.response_lock = threading.Lock()
        self.results = {}
        self.current_probing_target = None
        self.interface = None
        self.scanning = False
        self.scan_count = 0
        self.setup_csv_file()
        pub.subscribe(self.onReceive, "meshtastic.receive")
        self.timer = self.create_timer(PROBE_INTERVAL + 2, self.timer_callback, callback_group=self.callback_group)
        self.get_logger().info(f"FastScanNode TEXT_MESSAGE 版本啟動，輪詢 {len(TARGET_NODE_IDS)} 個節點")

    def setup_csv_file(self):
        file_exists = os.path.isfile(CSV_FILENAME)
        with open(CSV_FILENAME, mode='a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(["Timestamp", "TargetID", "Status", "RSSI", "SNR"])
        self.get_logger().info(f"CSV 檔案 '{CSV_FILENAME}' 準備就緒")

    def onReceive(self, packet, interface):
        try:
            port = packet.get('decoded', {}).get('portnum')
            from_id = packet.get('fromId', 'unknown')
            is_text = (port == "TEXT_MESSAGE_APP" or port == portnums_pb2.PortNum.TEXT_MESSAGE_APP)
            
            if is_text and from_id == self.current_probing_target:
                self.get_logger().info(f"[onReceive] ✓ 收到來自 {from_id} 的回應")
                with self.response_lock:
                    self.results['status'] = 'Success'
                    self.results['rssi'] = packet.get('rxRssi', 0)
                    self.results['snr'] = packet.get('rxSnr', 0)
                    self.response_event.set()
        except Exception as e:
            self.get_logger().error(f"[onReceive] 異常: {e}")

    def probe_node(self, target_id):
        self.get_logger().info(f"[probe_node] 開始探測 {target_id}")
        
        # 清理接收緩衝區
        self.get_logger().info(f"[probe_node] 等待 {PRE_PROBE_DELAY} 秒清理緩衝區...")
        for i in range(PRE_PROBE_DELAY):
            time.sleep(1)
            try:
                with self.response_lock:
                    self.response_event.clear()
            except:
                pass
        
        with self.response_lock:
            self.current_probing_target = target_id
            self.results = {'status': 'Timeout'}
            self.response_event.clear()
        
        try:
            self.interface.sendText("PROBE", destinationId=target_id, wantResponse=True)
            self.get_logger().info(f"[probe_node] TEXT_MESSAGE 探測已發送至 {target_id}")
        except Exception as e:
            self.get_logger().error(f"[probe_node] 發送失敗: {e}")
            return
        
        # 使用非阻塞輪詢等待回應
        start_time = time.time()
        poll_interval = 0.1
        response_received = False
        
        while time.time() - start_time < RESPONSE_TIMEOUT:
            if self.response_event.wait(timeout=poll_interval):
                response_received = True
                break
            time.sleep(0.05)
        
        if not response_received:
            self.get_logger().warning(f"[probe_node] {target_id} 無回應 (超時 {RESPONSE_TIMEOUT}s)")
        
        # 記錄結果
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        with self.response_lock:
            status = self.results['status']
            msg_data = {
                "timestamp": timestamp, "target_id": target_id, "status": status,
                "rssi": self.results.get('rssi', ''),
                "snr": self.results.get('snr', '')
            }
        
        # 寫入 CSV
        try:
            with open(CSV_FILENAME, mode='a', newline='', encoding='utf-8') as f:
                csv.writer(f).writerow([timestamp, target_id, status, 
                                       msg_data.get('rssi', ''), msg_data.get('snr', '')])
        except Exception as e:
            self.get_logger().error(f"CSV 寫入失敗: {e}")
        
        # 發布 ROS2 topic
        try:
            ros_msg = String()
            ros_msg.data = json.dumps(msg_data, ensure_ascii=False)
            self.link_pub.publish(ros_msg)
        except Exception as e:
            self.get_logger().warning(f"[probe_node] 發布失敗: {e}")
        
        self.get_logger().info(f"[probe_node] 結果: {status}")
        
        # 清理並延遲
        time.sleep(POST_PROBE_DELAY)
        try:
            if hasattr(self.interface, "flush"):
                self.interface.flush()
        except:
            pass
        
        self.get_logger().info(f"[probe_node] 探測 {target_id} 完成\n")

    def timer_callback(self):
        if self.scanning:
            return
        self.scanning = True
        self.scan_count += 1
        self.get_logger().info(f"\n{'='*60}\n開始第 {self.scan_count} 輪掃描\n{'='*60}")
        
        if not self.interface:
            try:
                self.interface = meshtastic.serial_interface.SerialInterface()
                self.get_logger().info("✓ Meshtastic interface 啟動")
            except Exception as e:
                self.get_logger().error(f"無法開啟 Meshtastic: {e}")
                self.scanning = False
                return
        
        for i, target_id in enumerate(TARGET_NODE_IDS):
            self.probe_node(target_id)
            if i < len(TARGET_NODE_IDS) - 1:
                self.get_logger().info(f"節點間延遲: 等待 {DELAY_BETWEEN_NODES} 秒...")
                time.sleep(DELAY_BETWEEN_NODES)
        
        self.get_logger().info(f"{'='*60}\n本輪掃描完成\n{'='*60}\n")
        self.scanning = False

    def destroy_node(self):
        self.get_logger().info("FastScanNode TEXT_MESSAGE 版本結束")
        if self.interface:
            try:
                self.interface.close()
            except:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FastScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("程式中斷")
    except Exception as e:
        node.get_logger().error(f"執行異常: {e}")
    finally:
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == "__main__":
    main()
