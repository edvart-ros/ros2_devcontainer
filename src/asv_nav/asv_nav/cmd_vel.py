import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import numpy as np
import math

MAX_LIN_VEL = 1.0
MAX_ANG_VEL = 0.5


def clamp(n, min, max): 
    if n < min: 
        return min
    elif n > max: 
        return max
    else: 
        return n 

class CmdVel(Node):

    def __init__(self):
        super().__init__('cmd_vel')
        self.odom = Odometry()
        self.force_pub = self.create_publisher(Twist, "cmd_force", 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.vel_cb, 1)
        self.odom_sub = self.create_subscription(Odometry, '/odometry', self.odom_cb, 1)
        self.rate = 50
        self.dt = 1/self.rate
        self.control_timer = self.create_timer(self.dt, self.controller_cb)
        self.K_up, self.K_ui, self.K_ud = 1.0, 0.8, 0.01
        self.K_rp, self.K_ri, self.K_rd = 1.0, 0.8, 0.01
        self.u, self.u_d, self.r, self.r_d, self.u_e_prev, self.r_e_prev = 0, 0, 0, 0, 0, 0
        self.u_e_acc = 0
        self.r_e_acc = 0
    

    def vel_cb(self, msg):
        # only linear.x and angular.z are controlled
        u_cmd = clamp(msg.linear.x, -1.0, 1.0)*MAX_LIN_VEL
        r_cmd = clamp(msg.angular.z, -1, 1.0)*MAX_ANG_VEL
        
        self.u_d = u_cmd
        self.r_d = r_cmd

    def odom_cb(self, msg):
        self.u = msg.twist.twist.linear.x
        self.r = msg.twist.twist.angular.z

    def controller_cb(self):
        # Surge-yaw PID control loop
        u_e = self.u_d - self.u
        r_e = self.r_d - self.r
        self.u_e_acc += ((u_e + self.u_e_prev) / 2) * self.dt
        self.r_e_acc += ((r_e + self.r_e_prev) / 2) * self.dt

        Fx = self.K_up * u_e + self.K_ud * ((u_e - self.u_e_prev) / self.dt) + self.K_ui * self.u_e_acc
        Mz = self.K_rp * r_e + self.K_rd * ((r_e - self.r_e_prev) / self.dt) + self.K_ri * self.r_e_acc

        self.force_pub.publish(Twist(linear=Vector3(x=Fx), angular=Vector3(z=Mz)))
        self.u_e_prev = u_e
        self.r_e_prev = r_e
        print(self.u_d, self.r_d)


def main():
    rclpy.init()
    node = CmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()