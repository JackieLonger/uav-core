#!/usr/bin/env python3
# 快速掃描節點 - 定期掃描 Meshtastic LoRa 節點並記錄連線品質
# 修正版：解決無法掃描第二個節點的問題 + 解決端口衝突問題

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String, Bool
import meshtastic
import meshtastic.serial_interface
from pubsub import pub
import time, threading
from datetime import datetime
from meshtastic.protobuf import mesh_pb2, portnums_pb2
import json, math

# 增加 Meshtastic 连接超时时间（从默认 20 秒增加到 60 秒）
import meshtastic.util
meshtastic.util.Timeout.__init__.__defaults__ = (60,)  # 设置默认超时为 60 秒

# LoRa 參數設置
LORA_BW_HZ = 125000
LORA_NF_DB = 6.0
THERMAL_NOISE_FLOOR = -174
NOISE_FLOOR_DBM = THERMAL_NOISE_FLOOR + 10 * math.log10(LORA_BW_HZ) + LORA_NF_DB

# 掃描設定 - 平衡版參數 (100% 成功率 + 優化耗時)
PROBE_INTERVAL = 8                # 掃描主循環間隔
DELAY_BETWEEN_NODES = 30          # 節點間延遲 (30 秒，足夠恢復)
RESPONSE_TIMEOUT = 45             # 回應超時 (45 秒)
PRE_PROBE_DELAY = 5               # 掃描前準備時間 (5 秒)
POST_PROBE_DELAY = 2              # 掃描後延遲 (2 秒)

class FastScanNode(Node):
    def __init__(self):
        super().__init__('fast_scan_node')
        
        # ===== 配置区域：每架无人机的 Jetson 需手动修改以下 Tracker ID =====
        # Tracker ID 格式：以 "!" 开头的 Meshtastic 节点 ID
        # 例如：TRACKER_A_ID = "!e2e5b7c4"
        #       TRACKER_B_ID = "!e2e5b8f8"
        TRACKER_A_ID = "!e2e5b7c4"  # ← 修改为实际的 Tracker A 节点 ID
        TRACKER_B_ID = "!e2e5b8f8"  # ← 修改为实际的 Tracker B 节点 ID
        # =====================================================================
        
        self.target_node_ids = [TRACKER_A_ID, TRACKER_B_ID]
        
        self.callback_group = ReentrantCallbackGroup()
        self.link_pub = self.create_publisher(String, 'link_quality', 10)
        
        # 扫描控制订阅（由 Laptop 远程控制）
        self.scan_control_sub = self.create_subscription(
            Bool,
            'scan_control',  # 相对名称，映射到 /drone_N/scan_control
            self.scan_control_callback,
            10
        )
        self.scan_enabled = False  # 默认关闭扫描，等待 Laptop 启动
        
        self.response_event = threading.Event()
        self.response_lock = threading.Lock()  # 線程安全保護
        self.results = {}
        self.current_probing_target = None
        self.interface = None
        self.scanning = False
        self.scan_count = 0
        # 直接绑定到 /dev/ttyACM0 (Heltec Wireless Tracker)
        self.meshtastic_port = '/dev/ttyACM0'
        pub.subscribe(self.onReceive, "meshtastic.receive")
        
        self.timer = self.create_timer(PROBE_INTERVAL + 2, self.timer_callback, callback_group=self.callback_group)
        self.get_logger().info(f"FastScanNode 啟動，綁定 Tracker: {self.target_node_ids}")
        self.get_logger().info(f"使用 Meshtastic 端口: {self.meshtastic_port} (固定綁定)")
        self.get_logger().info("扫描默认关闭，等待 Laptop 按 F 键启动")
    
    def scan_control_callback(self, msg: Bool):
        """接收扫描控制命令（由 Laptop 发送）"""
        self.scan_enabled = msg.data
        if self.scan_enabled:
            self.get_logger().info("📡 扫描已启动（收到远程命令）")
        else:
            self.get_logger().info("⏹️  扫描已停止（收到远程命令）")
    
    def onReceive(self, packet, interface):
        """
        處理 Meshtastic 回應封包
        
        資料來源：
        - return_rssi / return_snr：無人機直接量測（Tracker → 無人機）✅ 精確
        - forward_snr：從 traceroute 的 snr_towards 取得 ✅ 精確
        - forward_rssi：根據 SNR 差值估算（假設路徑損耗對稱）
        """
        try:
            port = packet.get('decoded', {}).get('portnum')
            from_id = packet.get('fromId', 'unknown')
            is_traceroute = (port == "TRACEROUTE_APP" or port == portnums_pb2.PortNum.TRACEROUTE_APP)
            
            if is_traceroute and from_id == self.current_probing_target:
                self.get_logger().info(f"[onReceive] ✓ 收到來自 {from_id} 的回應")
                with self.response_lock:
                    rd = mesh_pb2.RouteDiscovery()
                    try:
                        rd.ParseFromString(packet['decoded']['payload'])
                    except:
                        rd = None
                    
                    self.results['status'] = 'Success'
                    
                    # Return 方向：無人機直接量測（精確）- 確保是 float
                    self.results['return_rssi'] = float(packet.get('rxRssi', 0))
                    self.results['return_snr'] = float(packet.get('rxSnr', 0))
                    
                    # Forward 方向：從 traceroute snr_towards 取得
                    if rd and hasattr(rd, "snr_towards") and len(rd.snr_towards) > 0:
                        # snr_towards 是 1/4 dB 單位，需要除以 4
                        forward_snr = rd.snr_towards[-1] / 4.0
                        self.results['forward_snr'] = float(forward_snr)
                        # Forward RSSI：根據 SNR 差值估算（假設路徑損耗對稱）
                        self.results['forward_rssi'] = round(
                            self.results['return_rssi'] + (forward_snr - self.results['return_snr']), 1
                        )
                    else:
                        # 無 forward 資料時，使用 return 值作為備用（確保是 float）
                        self.get_logger().warn(f"[onReceive] {from_id} 無 snr_towards，使用 return 值代替")
                        self.results['forward_snr'] = self.results['return_snr']
                        self.results['forward_rssi'] = self.results['return_rssi']
                    
                    self.response_event.set()
        except Exception as e:
            self.get_logger().error(f"[onReceive] 異常: {e}")

    def probe_node(self, target_id):
        self.get_logger().info(f"[probe_node] 開始探測 {target_id}")
        
        # 清理接收緩衝區 - 用循環檢查和丟棄任何待處理的消息
        self.get_logger().info(f"[probe_node] 等待 {PRE_PROBE_DELAY} 秒清理緩衝區...")
        
        # 在緩衝區清理期間定期檢查和清除任何舊消息
        for i in range(PRE_PROBE_DELAY):
            time.sleep(1)
            # 強制清除任何陳舊的 response_event，以確保新的探測不會被舊回應幹擾
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
            self.interface.sendData(mesh_pb2.RouteDiscovery(), destinationId=target_id, 
                                   portNum=portnums_pb2.PortNum.TRACEROUTE_APP, wantResponse=True)
            self.get_logger().info(f"[probe_node] 探測請求已發送至 {target_id}")
        except Exception as e:
            self.get_logger().error(f"[probe_node] 發送失敗: {e}")
            return
        
        # 使用非阻塞輪詢等待回應 (避免阻塞 ROS2 上下文)
        start_time = time.time()
        poll_interval = 0.1  # 每 100ms 檢查一次
        response_received = False
        
        while time.time() - start_time < RESPONSE_TIMEOUT:
            if self.response_event.wait(timeout=poll_interval):
                response_received = True
                break
            # 短暫休眠讓其他執行緒執行
            time.sleep(0.05)
        
        if not response_received:
            self.get_logger().warning(f"[probe_node] {target_id} 無回應 (超時 {RESPONSE_TIMEOUT}s)")
        
        # 記錄結果
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        with self.response_lock:
            status = self.results['status']
            msg_data = {
                "timestamp": timestamp, "target_id": target_id, "status": status,
                "forward_rssi": self.results.get('forward_rssi', ''),
                "forward_snr": self.results.get('forward_snr', ''),
                "return_rssi": self.results.get('return_rssi', ''),
                "return_snr": self.results.get('return_snr', '')
            }
        
        
        # 發布 ROS2 topic
        try:
            ros_msg = String()
            ros_msg.data = json.dumps(msg_data, ensure_ascii=False)
            self.link_pub.publish(ros_msg)
        except Exception as e:
            self.get_logger().warning(f"[probe_node] 發布失敗: {e}")
        
        self.get_logger().info(f"[probe_node] 結果: {status}")
        
        # 清理並延遲
        time.sleep(POST_PROBE_DELAY)  # 使用配置的延遲
        try:
            if hasattr(self.interface, "flush"):
                self.interface.flush()
        except:
            pass
        
        self.get_logger().info(f"[probe_node] 探測 {target_id} 完成\n")

    def timer_callback(self):
        # 检查扫描是否启用
        if not self.scan_enabled:
            return
        
        if self.scanning:
            return
        self.scanning = True
        self.scan_count += 1
        self.get_logger().info(f"\n{'='*60}\n開始第 {self.scan_count} 輪掃描\n{'='*60}")
        
        # 每輪開始時重新初始化接口，確保狀態清潔（原始邏輯，100%成功率）
        if self.interface:
            try:
                self.interface.close()
                self.get_logger().info("已關閉舊的 Meshtastic 連接")
                time.sleep(2)  # 等待 2 秒確保連接完全關閉
            except:
                pass
            self.interface = None
        
        if not self.interface:
            try:
                self.interface = meshtastic.serial_interface.SerialInterface(devPath=self.meshtastic_port)
                self.get_logger().info("✓ Meshtastic interface 重新啟動")
                time.sleep(1)  # 等待 1 秒讓接口穩定
            except Exception as e:
                self.get_logger().error(f"無法開啟 Meshtastic: {e}")
                self.scanning = False
                return
        
        for i, target_id in enumerate(self.target_node_ids):
            self.probe_node(target_id)
            if i < len(self.target_node_ids) - 1:
                self.get_logger().info(f"節點間延遲: 等待 {DELAY_BETWEEN_NODES} 秒...")
                time.sleep(DELAY_BETWEEN_NODES)
        
        self.get_logger().info(f"{'='*60}\n本輪掃描完成\n{'='*60}\n")
        self.scanning = False

    def destroy_node(self):
        self.get_logger().info("FastScanNode 結束")
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
