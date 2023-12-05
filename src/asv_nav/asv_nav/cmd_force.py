import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np
import matplotlib.pyplot as plt
import cvxpy as cp

#rear left
L1_X = -2.316
L1_Y = 1.029
#rear right
L2_X = -2.316
L2_Y = -1.029
#front left
L3_X = 1.707
L3_Y = 1.029
#front right
L4_X = 1.707
L4_Y = -1.029

REAR_THRUSTER_ANGLE = np.pi/6 #rear thruster angle

B_r = np.array([[1, 0, 1, 0],
                [0, 0, 0, 1],
                [-L1_Y, L1_X, -L2_Y, L2_X]])
# configuration for the fixed front thrusters

B_f = np.array([[1, 0, 1, 0],
                [0, 0, 0, 1],
                [-L3_Y, L3_X, -L4_Y, L4_X]])

# matrix for azimuthing configuration
B_az = np.hstack((B_r, B_f))

B_PINV_AZ = np.linalg.pinv(B_az)
THRUST_LIMIT = 150
ANGLE_LIMIT = np.pi*0.5 #bi-directional

FORCE_X_MAX_AZ = 4*THRUST_LIMIT
FORCE_Y_MAX_AZ = 4*THRUST_LIMIT
MOMENT_Z_MAX_AZ = THRUST_LIMIT * (abs(L1_X) + abs(L1_Y) + abs(L2_X) + abs(L2_Y) + abs(L3_X) + abs(L3_Y) + abs(L4_X) + abs(L4_Y))
ANGLE_TOLERANCE = np.pi/2


class CmdForce(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_publisher')
        self.force_sub = self.create_subscription(Twist, "cmd_force", self.cmd_force_callback, 1)
        
        self.rl_thrust_pub = self.create_publisher(Float32, '/rear_left_thruster/thrust', 10)
        self.rl_dir_pub = self.create_publisher(Float32, '/rear_left_thruster/angle', 10)

        self.rr_thrust_pub = self.create_publisher(Float32, '/rear_right_thruster/thrust', 10)
        self.rr_dir_pub = self.create_publisher(Float32, '/rear_right_thruster/angle', 10)

        self.fl_thrust_pub = self.create_publisher(Float32, '/front_left_thruster/thrust', 10)
        self.fl_dir_pub = self.create_publisher(Float32, '/front_left_thruster/angle', 10)

        self.fr_thrust_pub = self.create_publisher(Float32, '/front_right_thruster/thrust', 10)
        self.fr_dir_pub = self.create_publisher(Float32, '/front_right_thruster/angle', 10)
    
    def publish_angle_commands(self, angles):
        angles = angles*(180/np.pi)
        self.rl_dir_pub.publish(Float32(data=angles[0]))
        self.rr_dir_pub.publish(Float32(data=angles[0]))
        self.fl_dir_pub.publish(Float32(data=angles[0]))
        self.fr_dir_pub.publish(Float32(data=angles[0]))

    def publish_thrust_commands(self, thrusts):
        self.rl_thrust_pub.publish(Float32(data=thrusts[0]))
        self.rr_thrust_pub.publish(Float32(data=thrusts[1]))
        self.fl_thrust_pub.publish(Float32(data=thrusts[2]))
        self.fr_thrust_pub.publish(Float32(data=thrusts[3]))

    def cmd_force_callback(self, msg):
        tau_desired = np.array([[FORCE_X_MAX_AZ*msg.linear.x],[FORCE_Y_MAX_AZ*msg.linear.y],[MOMENT_Z_MAX_AZ*msg.angular.z]]) #generalized desired force
        u = B_PINV_AZ @ tau_desired
        f = np.zeros(4)
        # rear
        f[0] = np.linalg.norm(u[0:2])
        f[1] = np.linalg.norm(u[2:4])
        # bow
        f[2] = np.linalg.norm(u[4:6])
        f[3] = np.linalg.norm(u[6:8])
        if np.max(abs(f)) > THRUST_LIMIT:
            f = f * THRUST_LIMIT / np.max(abs(f))

        #scale forces to [-1, 1] for publishing to unity thrusters
        f = f/THRUST_LIMIT
            
        angles = np.zeros(4)
        for i in range(4):
            angles[i] = np.arctan2(u[2*i+1,0], u[2*i,0])

        # if angle violated, flip direction and reverse thrust
        for i in range(len(angles)):
            if abs(angles[i]) > ANGLE_LIMIT:
                angles[i] += np.pi 
                angles[i] = np.arctan2(np.sin(angles[i]), np.cos(angles[i])) #keep in [-pi, pi]
                f[i] = -f[i]
            
        self.publish_angle_commands(angles)
        self.publish_thrust_commands(f)

def main():
    rclpy.init()
    node = CmdForce()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()