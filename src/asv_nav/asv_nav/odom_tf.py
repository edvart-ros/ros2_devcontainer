import math
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


BASE_LINK_FRAME = "base_link"
ODOM_FRAME = "odom"

class OdomTF(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_subscription = self.create_subscription(Odometry, "odometry", self.odom_cb,1)
        self.imu_sub = self.create_subscription(Imu, "imu/raw", self.imu_cb, 1)
        self.q = [0.0, 0.0, 0.0, 1.0]

    def odom_cb(self, msg):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = ODOM_FRAME
        t.child_frame_id = BASE_LINK_FRAME

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        t.transform.rotation.x = self.q[0]
        t.transform.rotation.y = self.q[1]
        t.transform.rotation.z = self.q[2]
        t.transform.rotation.w = self.q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

    def imu_cb(self, msg):
        self.q[0], self.q[1], self.q[2], self.q[3] = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w

def main():
    rclpy.init()
    node = OdomTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()