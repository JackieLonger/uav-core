
#!/usr/bin/env python3
"""
啟動 MicroXRCE-DDS Agent 連接 Pixhawk

此節點直接在前景執行 MicroXRCEAgent，適用於 SSH 無 GUI 環境。
"""

import subprocess
import sys
import signal
import rclpy
from rclpy.node import Node


class ProcessesNode(Node):
    """管理 MicroXRCE-DDS Agent 進程"""
    
    def __init__(self):
        super().__init__('processes_node')
        
        self.agent_process = None
        self.get_logger().info("🚀 啟動 MicroXRCE-DDS Agent...")
        
        # 啟動 MicroXRCEAgent
        try:
            self.agent_process = subprocess.Popen(
                ["MicroXRCEAgent", "serial", "-D", "/dev/ttyUSB0", "-b", "921600"],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True
            )
            self.get_logger().info("✅ MicroXRCE-DDS Agent 已啟動 (PID: {})".format(self.agent_process.pid))
        except FileNotFoundError:
            self.get_logger().error("❌ MicroXRCEAgent 未找到，請確認已安裝")
            return
        except Exception as e:
            self.get_logger().error(f"❌ 啟動 Agent 失敗: {e}")
            return
        
        # 創建定時器監控 Agent 狀態
        self.create_timer(5.0, self._check_agent_status)
    
    def _check_agent_status(self):
        """定期檢查 Agent 是否仍在運行"""
        if self.agent_process and self.agent_process.poll() is not None:
            self.get_logger().warning("⚠️  MicroXRCE-DDS Agent 已停止 (exit code: {})".format(
                self.agent_process.returncode))
    
    def destroy_node(self):
        """關閉時終止 Agent 進程"""
        if self.agent_process and self.agent_process.poll() is None:
            self.get_logger().info("🛑 停止 MicroXRCE-DDS Agent...")
            self.agent_process.terminate()
            try:
                self.agent_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.agent_process.kill()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ProcessesNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
