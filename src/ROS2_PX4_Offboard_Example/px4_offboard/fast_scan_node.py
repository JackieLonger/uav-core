#!/usr/bin/env python3
# 快速掃描節點 - 專業非同步執行緒版
# 整合安全鎖、事件化等待、以及即時中斷機制

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import meshtastic
import meshtastic.serial_interface
from pubsub import pub
import time, threading
from datetime import datetime
from meshtastic.protobuf import mesh_pb2, portnums_pb2
import json, math

# 增加 Meshtastic 連線超時時間
import meshtastic.util
meshtastic.util.Timeout.__init__.__defaults__ = (60,)

# 掃描設定
PROBE_INTERVAL = 8                # 輪次間隔
DELAY_BETWEEN_NODES = 30          # 節點間恢復延遲
RESPONSE_TIMEOUT = 45             # 探測回應超時
PRE_PROBE_DELAY = 5               # 探測前緩衝
POST_PROBE_DELAY = 2              # 探測後緩衝

class FastScanNode(Node):
    def __init__(self):
        super().__init__('fast_scan_node')
        
        # ===== 配置區域 =====
        self.target_node_ids = ["!e2e5b7c4", "!e2e5b8f8"]
        self.meshtastic_port = '/dev/ttyACM0'
        # ===================
        
        # 狀態控制與同步對象
        self.scan_enabled = False
        self.scanning_in_progress = False
        self.stop_event = threading.Event()      # 用於程式退出
        self.response_event = threading.Event()  # 用於等待封包回應
        self.interface_lock = threading.Lock()   # 保護序列埠操作
        self.response_lock = threading.Lock()    # 保護結果數據
        
        self.results = {}
        self.current_probing_target = None
        self.interface = None
        
        # ROS 通訊
        self.link_pub = self.create_publisher(String, 'link_quality', 10)
        self.scan_control_sub = self.create_subscription(
            Bool, 'scan_control', self.scan_control_callback, 10)
        
        # 註冊 Meshtastic 接收監聽
        pub.subscribe(self.onReceive, "meshtastic.receive")
        
        # 啟動獨立後台掃描執行緒
        self.worker_thread = threading.Thread(target=self.scan_worker_loop, daemon=True)
        self.worker_thread.start()
        
        self.get_logger().info(f"🚀 FastScanNode 啟動！目標: {self.target_node_ids}")
        self.get_logger().info("✅ 已採用背景執行緒與 Event-Wait 機制，不再阻塞控制迴圈。")

    def scan_control_callback(self, msg: Bool):
        """遠端控制掃描開關"""
        self.scan_enabled = msg.data
        if self.scan_enabled:
            self.get_logger().info("📡 掃描任務：啟動")
        else:
            self.get_logger().info("🛑 掃描任務：停止 (將在當前動作完成後中斷)")
            self.response_event.set() # 若正在等待回應，立即喚醒

    def onReceive(self, packet, interface):
        """處理回應封包 (來自 Meshtastic 內部執行緒)"""
        try:
            port = packet.get('decoded', {}).get('portnum')
            from_id = packet.get('fromId', 'unknown')
            is_traceroute = (port == "TRACEROUTE_APP" or port == portnums_pb2.PortNum.TRACEROUTE_APP)
            
            if is_traceroute and from_id == self.current_probing_target:
                with self.response_lock:
                    rd = mesh_pb2.RouteDiscovery()
                    try:
                        rd.ParseFromString(packet['decoded']['payload'])
                    except:
                        rd = None
                    
                    self.results['status'] = 'Success'
                    self.results['return_rssi'] = float(packet.get('rxRssi', 0))
                    self.results['return_snr'] = float(packet.get('rxSnr', 0))
                    
                    if rd and hasattr(rd, "snr_towards") and len(rd.snr_towards) > 0:
                        forward_snr = rd.snr_towards[-1] / 4.0
                        self.results['forward_snr'] = float(forward_snr)
                        self.results['forward_rssi'] = round(
                            self.results['return_rssi'] + (forward_snr - self.results['return_snr']), 1
                        )
                    else:
                        self.results['forward_snr'] = self.results['return_snr']
                        self.results['forward_rssi'] = self.results['return_rssi']
                    
                    self.response_event.set()
        except Exception as e:
            self.get_logger().error(f"[onReceive] 異常: {e}")

    def safe_wait(self, timeout):
        """✅ 安全等待：可被程式關閉或掃描停止立即喚醒"""
        return self.stop_event.wait(timeout=timeout)

    def scan_worker_loop(self):
        """背景執行緒主循環"""
        scan_count = 0
        while not self.stop_event.is_set() and rclpy.ok():
            if not self.scan_enabled:
                self.safe_wait(1.0)
                continue
            
            scan_count += 1
            self.get_logger().info(f"\n{'='*40}\n第 {scan_count} 輪掃描開始\n{'='*40}")
            
            # 1. 初始化或重啟介面
            with self.interface_lock:
                try:
                    if self.interface: 
                        self.interface.close()
                        self.safe_wait(2.0)
                    self.interface = meshtastic.serial_interface.SerialInterface(devPath=self.meshtastic_port)
                    self.get_logger().info("✓ Meshtastic 連接成功")
                except Exception as e:
                    self.get_logger().error(f"介面開啟失敗: {e}")
                    self.safe_wait(5.0)
                    continue

            # 2. 遍歷探測目標
            for i, target_id in enumerate(self.target_node_ids):
                if not self.scan_enabled or self.stop_event.is_set():
                    break
                
                self.probe_node_logic(target_id)
                
                # 節點間延遲
                if i < len(self.target_node_ids) - 1 and self.scan_enabled:
                    self.get_logger().info(f"等待恢復... ({DELAY_BETWEEN_NODES}s)")
                    self.safe_wait(DELAY_BETWEEN_NODES)
            
            self.get_logger().info(f"第 {scan_count} 輪掃描結束。")
            self.safe_wait(PROBE_INTERVAL)

    def probe_node_logic(self, target_id):
        """單點探測邏輯"""
        self.get_logger().info(f"[Probe] 清理緩衝區 ({PRE_PROBE_DELAY}s)...")
        for _ in range(PRE_PROBE_DELAY):
            if not self.scan_enabled: return
            self.safe_wait(1.0)
            self.response_event.clear()
        
        with self.response_lock:
            self.current_probing_target = target_id
            self.results = {'status': 'Timeout'}
            self.response_event.clear()
        
        # 發送探測
        try:
            with self.interface_lock:
                self.interface.sendData(mesh_pb2.RouteDiscovery(), destinationId=target_id, 
                                       portNum=portnums_pb2.PortNum.TRACEROUTE_APP, wantResponse=True)
            self.get_logger().info(f"[Probe] 已向 {target_id} 發送 Traceroute")
        except Exception as e:
            self.get_logger().error(f"[Probe] 發送失敗: {e}")
            self.publish_result(target_id, "Error")
            return

        # 等待回應 (Event-based, 不阻塞 ROS)
        if self.response_event.wait(timeout=RESPONSE_TIMEOUT) and self.scan_enabled:
            self.get_logger().info(f"✓ {target_id} 探測成功")
        else:
            self.get_logger().warning(f"✕ {target_id} 探測超時或被取消")
        
        self.publish_result(target_id, self.results.get('status', 'Timeout'))
        self.safe_wait(POST_PROBE_DELAY)

    def publish_result(self, target_id, status):
        """發布結果到 ROS Topic"""
        msg_data = {
            "timestamp": datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
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

    def destroy_node(self):
        """確保清理所有資源"""
        self.get_logger().info("正在關閉 FastScanNode...")
        self.stop_event.set()      # 喚醒所有執行緒等待
        self.scan_enabled = False
        self.response_event.set()  # 解除探測等待
        
        if self.worker_thread.is_alive():
            self.worker_thread.join(timeout=2.0)
            
        with self.interface_lock:
            if self.interface:
                try:
                    self.interface.close()
                    self.get_logger().info("Meshtastic 介面已關閉")
                except:
                    pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FastScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()