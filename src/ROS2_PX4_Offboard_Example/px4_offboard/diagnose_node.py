#!/usr/bin/env python3
"""診斷 Meshtastic 節點的詳細信息"""

import meshtastic
import meshtastic.serial_interface
from meshtastic.protobuf import portnums_pb2
import time

def diagnose():
    """診斷本地和遠端節點的信息"""
    try:
        interface = meshtastic.serial_interface.SerialInterface()
        
        # 獲取本地節點信息
        myinfo = interface.myInfo
        print("=== 本地節點信息 ===")
        print(f"節點編號: {myinfo.my_node_num}")
        print(f"設備型號: {myinfo.pio_env}")
        print(f"固件版本: {myinfo.min_app_version}")
        print(f"重啟次數: {myinfo.reboot_count}")
        
        # 查詢所有已知的節點
        print("\n=== 已知的遠端節點 ===")
        nodes = interface.nodesByNum
        for node_id, node_info in nodes.items():
            if node_id != myinfo.my_node_num:
                print(f"\n節點 ID: 0x{node_id:x} ({node_id})")
                if 'user' in node_info:
                    user = node_info['user']
                    print(f"  昵稱: {user.get('longName', 'N/A')}")
                if 'position' in node_info:
                    print(f"  位置已記錄")
                if 'lastHeard' in node_info:
                    print(f"  最後通訊: {node_info['lastHeard']}")
                if 'snr' in node_info:
                    print(f"  信噪比(SNR): {node_info['snr']}")
                if 'rssi' in node_info:
                    print(f"  接收強度(RSSI): {node_info['rssi']}")
                if 'hopStart' in node_info:
                    print(f"  初始跳跃數: {node_info['hopStart']}")
        
        # 測試連接
        print("\n=== 連接性測試 ===")
        target_ids = ["!e2e5b7c4", "!e2e5b8f8"]
        for target_id in target_ids:
            print(f"\n測試 {target_id}...")
            try:
                interface.sendData(
                    "TEST", 
                    destinationId=target_id,
                    portNum=portnums_pb2.PortNum.TEXT_MESSAGE_APP,
                    wantResponse=True
                )
                print(f"  已發送測試消息至 {target_id}")
                time.sleep(5)  # 等待回應
            except Exception as e:
                print(f"  發送失敗: {e}")
        
        interface.close()
        
    except Exception as e:
        print(f"診斷失敗: {e}")

if __name__ == "__main__":
    diagnose()
