#!/usr/bin/env python3
import math, random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from meshtastic_ros_bridge_msgs.msg import MeshtasticLinkQualityArray
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint

def now_us(node: Node) -> int:
    return int(node.get_clock().now().nanoseconds / 1000)

class SignalAscent(Node):
    def __init__(self):
        super().__init__('signal_ascent_controller')
        self.ns = self.declare_parameter('ns', 'uav1').value
        self.backend = self.declare_parameter('backend', 'px4').value
        self.center_x = self.declare_parameter('center_x', 0.0).value
        self.center_y = self.declare_parameter('center_y', 0.0).value
        self.radius   = self.declare_parameter('radius', 1.5).value
        self.delta = self.declare_parameter('delta', 0.25).value
        self.eta   = self.declare_parameter('eta', 0.6).value
        self.min_speed = self.declare_parameter('min_speed', 0.05).value
        self.max_speed = self.declare_parameter('max_speed', 0.3).value
        self.cmd_rate = self.declare_parameter('cmd_rate', 20.0).value

        self.curr_x = 0.0
        self.curr_y = 0.0
        self.latest_q = {}

        self.sub_pose = self.create_subscription(PoseStamped, f'/{self.ns}/local_position/pose', self.pose_cb, 10)
        self.sub_lq = self.create_subscription(MeshtasticLinkQualityArray, f'/{self.ns}/meshtastic/link_quality', self.lq_cb, 10)

        if self.backend == 'mavros':
            self.pub_twist = self.create_publisher(TwistStamped, f'/{self.ns}/mavros/setpoint_velocity/cmd_vel', 10)
        else:
            self.pub_offb = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
            self.pub_sp   = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)

        self.timer = self.create_timer(1.0 / self.cmd_rate, self.loop)
        self.phase = 0
        self.dir_xy = (1.0, 0.0)

    def pose_cb(self, msg: PoseStamped):
        self.curr_x = msg.pose.position.x
        self.curr_y = msg.pose.position.y

    def lq_cb(self, arr: MeshtasticLinkQualityArray):
        self.latest_q = {l.id: float(l.q) for l in arr.links}

    def inside(self, x, y): return (x-self.center_x)**2+(y-self.center_y)**2 <= self.radius**2
    def sat(self,v,vmin,vmax): return 0.0 if abs(v)<vmin else max(-vmax,min(vmax,v))
    def objective(self): return sum(self.latest_q.values()) if self.latest_q else 0.0

    def send_velocity_cmd(self,vx,vy):
        if self.backend=='mavros':
            msg=TwistStamped(); msg.header.stamp=self.get_clock().now().to_msg()
            msg.twist.linear.x=vx; msg.twist.linear.y=vy; self.pub_twist.publish(msg); return
        offb=OffboardControlMode(); offb.timestamp=now_us(self); offb.velocity=True
        self.pub_offb.publish(offb)
        sp=TrajectorySetpoint(); sp.timestamp=offb.timestamp
        sp.vx=vy; sp.vy=vx; sp.vz=0.0
        sp.position=[float('nan')]*3; sp.yaw=float('nan')
        self.pub_sp.publish(sp)

    def loop(self):
        if not self.inside(self.curr_x,self.curr_y):
            vx=self.sat(self.center_x-self.curr_x,0.05,self.max_speed)
            vy=self.sat(self.center_y-self.curr_y,0.05,self.max_speed)
            self.send_velocity_cmd(vx,vy); self.phase=0; return
        if self.phase==0:
            th=random.uniform(0,2*math.pi); self.dir_xy=(math.cos(th),math.sin(th)); self.phase=1; return
        dx,dy=self.dir_xy; δ=self.delta
        if self.phase==1:
            self.send_velocity_cmd(self.sat(dx,0.05,self.max_speed),self.sat(dy,0.05,self.max_speed))
            self.phase=2; return
        if self.phase==2:
            f=self.objective()
            vx=self.sat(self.eta*f*dx,0.05,self.max_speed)
            vy=self.sat(self.eta*f*dy,0.05,self.max_speed)
            self.send_velocity_cmd(vx,vy); self.phase=0; return

def main():
    rclpy.init()
    n=SignalAscent()
    rclpy.spin(n)
    n.destroy_node(); rclpy.shutdown()

if __name__=='__main__':
    main()
