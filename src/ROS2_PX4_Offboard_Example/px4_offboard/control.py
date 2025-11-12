#!/usr/bin/env python3
import sys
import threading

import geometry_msgs.msg
import rclpy
import std_msgs.msg

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty
    import select


msg = """
Keyboard teleop for offboard velocity setpoints.

W: Up (Z+)
S: Down (Z-)
A: Yaw Left  (Yaw-)
D: Yaw Right (Yaw+)
Y: HOLD (IMMEDIATE STOP/HOVER)  
Up Arrow:    Pitch Forward (Y+)
Down Arrow:  Pitch Back    (Y-)
Left Arrow:  Roll Left     (X+)
Right Arrow: Roll Right    (X-)

Press SPACE to arm/disarm the drone
"""

# (x, y, z, yaw) increments per keypress（累加 setpoint）
moveBindings = {
    'w': (0, 0, 1, 0),     # Z+
    's': (0, 0, -1, 0),    # Z-
    'a': (0, 0, 0, -1),    # Yaw-
    'd': (0, 0, 0, 1),     # Yaw+
    'y': (0, 0, 0, 0),     # HOLD（特殊處理）
    '\x1b[A': (0, 1, 0, 0),    # Up Arrow    -> Y+
    '\x1b[B': (0, -1, 0, 0),   # Down Arrow  -> Y-
    '\x1b[C': (-1, 0, 0, 0),   # Right Arrow -> X-
    '\x1b[D': (1, 0, 0, 0),    # Left Arrow  -> X+
}


def getKey_nonblocking(old_settings):
    """非阻塞式單鍵讀取（*nix）；Windows 用 thread 包住阻塞式讀取。"""
    if sys.platform == 'win32':
        if msvcrt.kbhit():
            ch = msvcrt.getwch()
            return ch
        return None
    else:
        tty.setraw(sys.stdin.fileno())
        dr, _, _ = select.select([sys.stdin], [], [], 0.01)
        key = None
        if dr:
            key = sys.stdin.read(1)
            if key == '\x1b':  # arrow key prefix
                additional_chars = sys.stdin.read(2)
                key += additional_chars
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    old_settings = saveTerminalSettings()
    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')

    
    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )

    pub = node.create_publisher(geometry_msgs.msg.Twist, '/offboard_velocity_cmd', qos_profile)
    arm_pub = node.create_publisher(std_msgs.msg.Bool, '/arm_message', qos_profile)
    hold_pub = node.create_publisher(std_msgs.msg.Bool, '/hold_message', qos_profile)

    speed = 0.1
    turn = 0.2

    # 累積 setpoint
    x_val = 0.0
    y_val = 0.0
    z_val = 0.0
    yaw_val = 0.0

    # 共享狀態（鍵盤執行緒更新、timer 讀取）
    state_lock = threading.Lock()
    hold_mode = {'value': False}
    arm_toggle = {'value': False}

    print(msg)

    def print_setpoints(x, y, z, yaw, hold=False):
        tag = " [HOLD]" if hold else ""
        print(f"X:{x:.3f}  Y:{y:.3f}  Z:{z:.3f}  Yaw:{yaw:.3f}{tag}")

    def publish_once(x, y, z, yaw):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.linear.z = z
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = yaw
        pub.publish(twist)

    def keyboard_thread():
        nonlocal x_val, y_val, z_val, yaw_val
        while rclpy.ok():
            key = getKey_nonblocking(old_settings)
            if key is None:
                continue

            # Ctrl+C
            if key == '\x03':
                rclpy.shutdown()
                return

            # Arm/Disarm
            if key == ' ':
                with state_lock:
                    arm_toggle['value'] = not arm_toggle['value']
                msg_arm = std_msgs.msg.Bool()
                msg_arm.data = arm_toggle['value']
                arm_pub.publish(msg_arm)
                print(f"[ARM] toggle -> {arm_toggle['value']}")
                continue

            # HOLD：清零 + 鎖定 hold（timer 會持續送 0），只在此刻列印與送一次 0
            if key == 'y':
                with state_lock:
                    x_val = y_val = z_val = yaw_val = 0.0
                    hold_mode['value'] = True

                hold_msg = std_msgs.msg.Bool()
                hold_msg.data = True
                hold_pub.publish(hold_msg)

                publish_once(0.0, 0.0, 0.0, 0.0)
                print_setpoints(0.0, 0.0, 0.0, 0.0, hold=True)
                continue

            # 方向鍵/WSAD：解除 hold 並累加 setpoint；只在按鍵時列印與送一次
            if key in moveBindings:
                dx, dy, dz, dyaw = moveBindings[key]
                with state_lock:
                    hold_mode['value'] = False
                    x_val += dx * speed
                    y_val += dy * speed
                    z_val += dz * speed
                    yaw_val += dyaw * turn

                publish_once(x_val, y_val, z_val, yaw_val)
                print_setpoints(x_val, y_val, z_val, yaw_val)
            # 其他鍵忽略

    # 啟動鍵盤讀取執行緒（只在按鍵時印出/送一次；連續發佈交給 timer）
    t = threading.Thread(target=keyboard_thread, daemon=True)
    t.start()

    # 20 Hz 連續發佈目前 setpoint（HOLD 時持續發 0）；不列印
    def publish_timer_cb():
        nonlocal x_val, y_val, z_val, yaw_val
        with state_lock:
            if hold_mode['value']:
                vx = vy = vz = vyaw = 0.0
            else:
                vx, vy, vz, vyaw = x_val, y_val, z_val, yaw_val
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.linear.z = vz
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = vyaw
        pub.publish(twist)

    timer = node.create_timer(1.0 / 20.0, publish_timer_cb)  # 20 Hz

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 結束前送一筆全零
        twist = geometry_msgs.msg.Twist()
        pub.publish(twist)
        restoreTerminalSettings(old_settings)
        node.destroy_timer(timer)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
