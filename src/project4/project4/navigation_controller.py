import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from project4.disc_robot import load_disc_robot

import numpy as np
import math

class NavigationController(Node):

    def __init__(self):
        super().__init__('NavigationController')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_file', "STRING")
            ]
        )

        self.robot = load_disc_robot(self.get_parameter('robot_file').value)
        self.danger = self.robot['body']['radius'] + 0.10

        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.scan_subscriber = self.create_subscription(LaserScan, "/scan", self.process_lidar, 10)

    # process lidar information and choose the best angular and linear velocity to send to /cmd_vel
    def process_lidar(self, msg):
        curr_angle = msg.angle_min  

        # vector component values
        vec_x = 0
        vec_y = 0

        # proportional measures for 
        lin_weight = 0.008
        ang_weight = 1

        # sum all vectors from sensor to range point
        for dist in msg.ranges:

            # if distance is infinity, ignore vector
            if math.isnan(dist):
                curr_angle += msg.angle_increment
                continue

            # add more weight to vectors that are in the "danger zone"
            if dist < self.danger:
                vec_x -= dist * np.cos(curr_angle) * 2
                vec_y -= dist * np.sin(curr_angle) * 2
            else:
                vec_x += dist * np.cos(curr_angle)
                vec_y += dist * np.sin(curr_angle)

            curr_angle += msg.angle_increment

        # set velocities proportional to angle and magnitude of vector
        twist_msg = Twist()
        twist_msg.angular.z = ang_weight * np.arctan2(vec_y, vec_x)
        twist_msg.linear.x = lin_weight * np.sqrt(vec_x**2 + vec_y**2)

        self.cmd_vel_publisher.publish(twist_msg)


def main():
    rclpy.init()
    my_node = NavigationController()
    rclpy.spin(my_node)

    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__init__":
    main()