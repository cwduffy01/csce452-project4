import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class NavigationController(Node):

    def __init__(self):
        super().__init__('NavigationController')

        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.scan_subscriber = self.create_subscription(LaserScan, "/scan", self.process_lidar, 10)

    # process lidar information and choose the best angular and linear velocity to send to /cmd_vel
    def process_lidar(self, msg):
        pass


def main():
    rclpy.init()
    my_node = NavigationController()
    rclpy.spin(my_node)

    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__init__":
    main()