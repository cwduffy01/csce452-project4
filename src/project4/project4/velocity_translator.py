import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class VelocityTranslator(Node):

    def __init__(self):
        super().__init__('VelocityTranslator')

        self.vl_publisher = self.create_publisher(Float64, "/vl", 10)
        self.vr_publisher = self.create_publisher(Float64, "/vr", 10)

        self.cmd_vel_subscriber = self.create_publisher(Twist, "/cmd_vel", self.translate_vel, 10)

    # translate Twist message from cmd_vel to Float64 messages to /vl and /vr
    def translate_vel(self, msg):
        pass


def main():
    rclpy.init()
    my_node = VelocityTranslator()
    rclpy.spin(my_node)

    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__init__":
    main()