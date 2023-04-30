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
        self.danger = self.robot['body']['radius'] + 0.1

        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.scan_subscriber = self.create_subscription(LaserScan, "/scan", self.process_lidar, 10)

    # process lidar information and choose the best angular and linear velocity to send to /cmd_vel
    def process_lidar(self, msg):

        curr_angle = msg.angle_min

        x_weight = 0
        y_weight = 0

        lin_weight = 0.01
        ang_weight = 1

        for dist in msg.ranges:
            angle = curr_angle

            if math.isnan(dist):
                curr_angle += msg.angle_increment
                continue

            if dist < self.danger:
                angle *= -1

            x_weight += dist * np.cos(angle)
            y_weight += dist * np.sin(angle)

            curr_angle += msg.angle_increment

        self.get_logger().info(f"x_weight: {x_weight}\ty_weight: {y_weight}\ttangent:{np.arctan2(y_weight, x_weight)}")
        




        twist_msg = Twist()
        twist_msg.angular.z = ang_weight * np.arctan2(y_weight, x_weight)
        twist_msg.linear.x = lin_weight * np.sqrt(x_weight**2 + y_weight**2)

        self.cmd_vel_publisher.publish(twist_msg)


def main():
    rclpy.init()
    my_node = NavigationController()
    rclpy.spin(my_node)

    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__init__":
    main()