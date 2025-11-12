#!/usr/bin/env python3
"""
進階版快速掃描節點 - 用於韌體升級 v2.7.11 後的雙節點掃描
修正第二個節點無回應的問題，包含詳細診斷和極端條件測試
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
import meshtastic
import meshtastic.serial_interface
from pubsub import pub
import time, threading, csv, os, json
from datetime import datetime
from meshtastic.protobuf import mesh_pb2, portnums_pb2
import math

# LoRa 參數設置
LORA_BW_HZ = 125000
LORA_NF_DB = 6.0
THERMAL_NOISE_FLOOR = -174
NOISE_FLOOR_DBM = THERMAL_NOISE_FLOOR + 10 * math.log10(LORA_BW_HZ) + LORA_NF_DB

# 進階掃描設定 - 用於 v2.7.11 韌體升級後
TARGET_NODE_IDS = ["!e2e5b7c4", "!e2e5b8f8"]
PROBE_INTERVAL = 8              # 增加至 8 秒（原 5 秒）
DELAY_BETWEEN_NODES = 30        # 進階版：30 秒（原 20 秒）- 給予更多通信時間
RESPONSE_TIMEOUT = 60           # 進階版：60 秒（原 45 秒）- 極端情況下等待更久
PRE_PROBE_DELAY = 5             # 進階版：5 秒（原 3 秒）- 更久的緩衝清理時間
POST_PROBE_DELAY = 3            # 新增：掃描後延遲（固化狀態）
INTERFACE_RESET_INTERVAL = 5    # 每 5 輪掃描重置一次 interface
CSV_FILENAME = "fast_scan_log_advanced.csv"
DIAG_LOG_FILENAME = "fast_scan_diagnostics.log"


class FastScanNodeAdvanced(Node):
    def __init__(self):
        super().__init__('fast_scan_node_advanced')
        self.callback_group = ReentrantCallbackGroup()
        self.link_pub = self.create_publisher(String, 'link_quality', 10)
        self.response_event = threading.Event()
        self.response_lock = threading.Lock()
        self.interface_lock = threading.Lock()
        self.results = {}
        self.current_probing_target = None
        self.interface = None
        self.scanning = False
        self.scan_count = 0
        self.probe_count = 0
        self.success_count = 0
        
        # 詳細診斷追蹤
        self.diagnostic_data = {
            "node_success_rate": {},
            "node_last_response_time": {},
            "node_consecutive_timeouts": {},
            "interface_reset_count": 0
        }
        
        self.setup_csv_file()
        self.setup_diagnostic_log()
        pub.subscribe(self.onReceive, "meshtastic.receive")
        self.timer = self.create_timer(PROBE_INTERVAL + 3, self.timer_callback, callback_group=self.callback_group)
        self.get_logger().info(f"FastScanNodeAdvanced 啟動 (v2.7.11 進階版)")
        self.get_logger().info(f"掃描目標: {TARGET_NODE_IDS}")
        self.get_logger().info(f"掃描間隔: {PROBE_INTERVAL}s, 節點間延遲: {DELAY_BETWEEN_NODES}s, 超時: {RESPONSE_TIMEOUT}s")

    def setup_csv_file(self):
        file_exists = os.path.isfile(CSV_FILENAME)
        with open(CSV_FILENAME, mode='a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(["Timestamp", "TargetID", "Status", "Forward_RSSI", "Forward_SNR", 
                               "Return_RSSI", "Return_SNR", "Response_Time_ms", "Interface_Health"])
        self.get_logger().info(f"CSV 檔案 '{CSV_FILENAME}' 準備就緒")

    def setup_diagnostic_log(self):
        with open(DIAG_LOG_FILENAME, mode='w', encoding='utf-8') as f:
            f.write(f"=== Meshtastic 進階掃描診斷日誌 ===\n")
            f.write(f"啟動時間: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"掃描節點: {TARGET_NODE_IDS}\n")
            f.write(f"韌體期望版本: v2.7.11\n")
            f.write(f"{'='*50}\n\n")
        self.get_logger().info(f"診斷日誌 '{DIAG_LOG_FILENAME}' 建立")

    def log_diagnostic(self, message):
        """記錄診斷信息"""
        with open(DIAG_LOG_FILENAME, mode='a', encoding='utf-8') as f:
            f.write(f"[{datetime.now().strftime('%H:%M:%S')}] {message}\n")

    def onReceive(self, packet, interface):
        """處理 Meshtastic 接收的回應包"""
        try:
            port = packet.get('decoded', {}).get('portnum')
            from_id = packet.get('fromId', 'unknown')
            is_traceroute = (port == "TRACEROUTE_APP" or port == portnums_pb2.PortNum.TRACEROUTE_APP)
            
            if is_traceroute and from_id == self.current_probing_target:
                response_time_ms = (datetime.now() - self.probe_start_time).total_seconds() * 1000
                self.get_logger().info(f"[onReceive] ✓ 收到來自 {from_id} 的回應 (耗時 {response_time_ms:.1f}ms)")
                self.log_diagnostic(f"[SUCCESS] 從 {from_id} 接收回應，耗時 {response_time_ms:.1f}ms")
                
                with self.response_lock:
                    rd = mesh_pb2.RouteDiscovery()
                    try:
                        rd.ParseFromString(packet['decoded']['payload'])
                    except:
                        rd = None
                    
                    self.results['status'] = 'Success'
                    self.results['response_time_ms'] = response_time_ms
                    self.results['return_rssi'] = packet.get('rxRssi', 0)
                    self.results['return_snr'] = packet.get('rxSnr', 0)
                    
                    if rd and hasattr(rd, "snr_towards") and len(rd.snr_towards) > 0:
                        forward_snr = rd.snr_towards[-1] / 4.0
                        self.results['forward_snr'] = forward_snr
                        self.results['forward_rssi'] = round(self.results['return_rssi'] + (forward_snr - self.results['return_snr']), 1)
                    else:
                        self.results['forward_snr'] = 'N/A'
                        self.results['forward_rssi'] = 'N/A'
                    
                    # 更新診斷數據
                    if from_id not in self.diagnostic_data["node_success_rate"]:
                        self.diagnostic_data["node_success_rate"][from_id] = 0
                        self.diagnostic_data["node_consecutive_timeouts"][from_id] = 0
                    self.diagnostic_data["node_success_rate"][from_id] += 1
                    self.diagnostic_data["node_consecutive_timeouts"][from_id] = 0
                    self.diagnostic_data["node_last_response_time"][from_id] = datetime.now()
                    
                    self.response_event.set()
        except Exception as e:
            self.get_logger().error(f"[onReceive] 異常: {e}")
            self.log_diagnostic(f"[ERROR] onReceive 異常: {e}")

    def probe_node(self, target_id):
        """探測單個節點，包含詳細診斷"""
        self.probe_start_time = datetime.now()
        self.get_logger().info(f"[probe_node] 開始探測 {target_id}")
        self.log_diagnostic(f"[PROBE_START] 開始探測 {target_id}")
        
        # 進階版：更久的緩衝區清理
        self.get_logger().info(f"[probe_node] 等待 {PRE_PROBE_DELAY} 秒清理緩衝區...")
        for i in range(PRE_PROBE_DELAY):
            time.sleep(1)
            self.get_logger().debug(f"  [{i+1}/{PRE_PROBE_DELAY}]")
        
        with self.response_lock:
            self.current_probing_target = target_id
            self.results = {'status': 'Timeout', 'interface_health': 'OK'}
            self.response_event.clear()
        
        try:
            # 檢查 interface 健康狀況
            if not self.interface or not hasattr(self.interface, 'myInfo'):
                self.get_logger().warning("[probe_node] Interface 狀態異常，嘗試重新連接...")
                self.log_diagnostic("[WARNING] Interface 健康檢查失敗")
                with self.interface_lock:
                    if self.interface:
                        try:
                            self.interface.close()
                        except:
                            pass
                    self.interface = meshtastic.serial_interface.SerialInterface()
                    self.get_logger().info("✓ Interface 重新連接")
                    self.log_diagnostic("[RECOVERY] Interface 已重新連接")
            
            # 發送 traceroute 請求
            self.interface.sendData(mesh_pb2.RouteDiscovery(), destinationId=target_id, 
                                   portNum=portnums_pb2.PortNum.TRACEROUTE_APP, wantResponse=True)
            self.get_logger().info(f"[probe_node] 探測請求已發送至 {target_id}")
            self.log_diagnostic(f"[SENT] Traceroute 請求已發送至 {target_id}")
            
        except Exception as e:
            self.get_logger().error(f"[probe_node] 發送失敗: {e}")
            self.log_diagnostic(f"[ERROR] 發送失敗: {e}")
            with self.response_lock:
                self.results['interface_health'] = 'ERROR'
            return
        
        # 等待回應 (進階版：60 秒超時)
        timeout_occurred = not self.response_event.wait(timeout=RESPONSE_TIMEOUT)
        
        if timeout_occurred:
            self.get_logger().warning(f"[probe_node] {target_id} 無回應 (超時 {RESPONSE_TIMEOUT}s)")
            self.log_diagnostic(f"[TIMEOUT] {target_id} 無回應 (超時 {RESPONSE_TIMEOUT}s)")
            if target_id not in self.diagnostic_data["node_consecutive_timeouts"]:
                self.diagnostic_data["node_consecutive_timeouts"][target_id] = 0
            self.diagnostic_data["node_consecutive_timeouts"][target_id] += 1
        else:
            self.get_logger().info(f"[probe_node] ✓ {target_id} 已回應")
        
        # 記錄結果
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        with self.response_lock:
            status = self.results['status']
            msg_data = {
                "timestamp": timestamp, 
                "target_id": target_id, 
                "status": status,
                "forward_rssi": self.results.get('forward_rssi', ''),
                "forward_snr": self.results.get('forward_snr', ''),
                "return_rssi": self.results.get('return_rssi', ''),
                "return_snr": self.results.get('return_snr', ''),
                "response_time_ms": self.results.get('response_time_ms', ''),
                "interface_health": self.results.get('interface_health', '')
            }
        
        # 寫入 CSV (包含響應時間和 interface 健康狀態)
        try:
            with open(CSV_FILENAME, mode='a', newline='', encoding='utf-8') as f:
                csv.writer(f).writerow([
                    timestamp, target_id, status, 
                    msg_data.get('forward_rssi', ''), 
                    msg_data.get('forward_snr', ''),
                    msg_data.get('return_rssi', ''), 
                    msg_data.get('return_snr', ''),
                    msg_data.get('response_time_ms', ''),
                    msg_data.get('interface_health', '')
                ])
        except Exception as e:
            self.get_logger().error(f"CSV 寫入失敗: {e}")
        
        # 發布 ROS2 topic
        ros_msg = String()
        ros_msg.data = json.dumps(msg_data, ensure_ascii=False)
        self.link_pub.publish(ros_msg)
        self.get_logger().info(f"[probe_node] 結果: {status}")
        
        # 進階版：掃描後延遲 (固化狀態)
        self.get_logger().info(f"[probe_node] 掃描後延遲 {POST_PROBE_DELAY} 秒...")
        time.sleep(POST_PROBE_DELAY)
        
        # 進階版：嘗試 flush buffer
        try:
            if hasattr(self.interface, "flush"):
                self.interface.flush()
                self.get_logger().debug("[probe_node] Interface buffer 已 flush")
        except:
            pass
        
        self.get_logger().info(f"[probe_node] 探測 {target_id} 完成\n")
        self.log_diagnostic(f"[COMPLETE] 探測 {target_id} 完成，狀態: {status}")

    def timer_callback(self):
        """定期掃描回調"""
        if self.scanning:
            return
        
        self.scanning = True
        self.scan_count += 1
        
        # 進階版：每 5 輪重置一次 interface
        if self.scan_count > 1 and self.scan_count % INTERFACE_RESET_INTERVAL == 0:
            self.get_logger().info("[timer_callback] 執行定期 interface 重置...")
            self.log_diagnostic("[RESET] 執行定期 interface 重置")
            with self.interface_lock:
                if self.interface:
                    try:
                        self.interface.close()
                        time.sleep(2)
                    except:
                        pass
                self.interface = None
            self.diagnostic_data["interface_reset_count"] += 1
        
        self.get_logger().info(f"\n{'='*70}")
        self.get_logger().info(f"開始第 {self.scan_count} 輪掃描 (總 probe: {self.probe_count}, 成功: {self.success_count})")
        self.get_logger().info(f"{'='*70}")
        self.log_diagnostic(f"\n[SCAN_START] 第 {self.scan_count} 輪掃描開始")
        
        # 初始化 interface
        if not self.interface:
            try:
                self.interface = meshtastic.serial_interface.SerialInterface()
                self.get_logger().info("✓ Meshtastic interface 啟動")
                self.log_diagnostic("[INIT] Meshtastic interface 已啟動")
            except Exception as e:
                self.get_logger().error(f"無法開啟 Meshtastic: {e}")
                self.log_diagnostic(f"[ERROR] 無法開啟 Meshtastic: {e}")
                self.scanning = False
                return
        
        # 掃描所有節點
        for i, target_id in enumerate(TARGET_NODE_IDS):
            self.probe_node(target_id)
            self.probe_count += 1
            
            # 檢查是否成功
            if self.results.get('status') == 'Success':
                self.success_count += 1
            
            # 節點間延遲
            if i < len(TARGET_NODE_IDS) - 1:
                self.get_logger().info(f"節點間延遲: 等待 {DELAY_BETWEEN_NODES} 秒...")
                self.log_diagnostic(f"[DELAY] 等待 {DELAY_BETWEEN_NODES} 秒")
                for j in range(DELAY_BETWEEN_NODES):
                    time.sleep(1)
        
        # 掃描完成統計
        success_rate = (self.success_count / self.probe_count * 100) if self.probe_count > 0 else 0
        self.get_logger().info(f"{'='*70}")
        self.get_logger().info(f"本輪掃描完成 | 成功率: {success_rate:.1f}% ({self.success_count}/{self.probe_count})")
        self.get_logger().info(f"節點診斷:")
        for node_id, timeouts in self.diagnostic_data["node_consecutive_timeouts"].items():
            success = self.diagnostic_data["node_success_rate"].get(node_id, 0)
            self.get_logger().info(f"  {node_id}: 成功 {success} 次, 連續超時 {timeouts} 次")
        self.get_logger().info(f"Interface 重置次數: {self.diagnostic_data['interface_reset_count']}")
        self.get_logger().info(f"{'='*70}\n")
        self.log_diagnostic(f"[SCAN_COMPLETE] 成功率: {success_rate:.1f}% ({self.success_count}/{self.probe_count})")
        
        self.scanning = False

    def destroy_node(self):
        self.get_logger().info("FastScanNodeAdvanced 結束")
        self.log_diagnostic(f"\n[SHUTDOWN] 程式結束時間: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        if self.interface:
            try:
                self.interface.close()
            except:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FastScanNodeAdvanced()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("程式中斷")
        node.log_diagnostic("[INTERRUPT] 使用者中斷程式")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
