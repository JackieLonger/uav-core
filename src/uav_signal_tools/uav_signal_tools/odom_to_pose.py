#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from px4_msgs.msg import VehicleOdometry

def q_from_yaw(yaw):
    half = yaw * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))

class OdomToPose(Node):
    def __init__(self):
        super().__init__('odom_to_pose')
        self.ns = self.declare_parameter('ns', 'uav1').value
        self.frame_id = self.declare_parameter('frame_id', 'map').value
        self.sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.cb, 10)
        self.pub = self.create_publisher(PoseStamped, f'/{self.ns}/local_position/pose', 10)

    def cb(self, msg: VehicleOdometry):
        # NED -> ENU
        x_enu = msg.position[1]
        y_enu = msg.position[0]
        z_enu = -msg.position[2]
        vx, vy = msg.velocity[0], msg.velocity[1]
        yaw_ned = math.atan2(vy, vx) if (abs(vx)+abs(vy)) > 1e-3 else 0.0
        yaw_enu = yaw_ned + math.pi/2.0

        p = PoseStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = self.frame_id
        p.pose.position.x = float(x_enu)
        p.pose.position.y = float(y_enu)
        p.pose.position.z = float(z_enu)
        p.pose.orientation = q_from_yaw(yaw_enu)
        self.pub.publish(p)

def main():
    rclpy.init()
    n = OdomToPose()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
