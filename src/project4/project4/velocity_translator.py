import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

from project4.disc_robot import load_disc_robot

class VelocityTranslator(Node):

    def __init__(self):
        super().__init__('VelocityTranslator')

        # pull parameters from command line 
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_file', "STRING"),
            ]
        )

        self.robot = load_disc_robot(self.get_parameter('robot_file').value)

        self.vl_publisher = self.create_publisher(Float64, "/vl", 10)
        self.vr_publisher = self.create_publisher(Float64, "/vr", 10)

        self.cmd_vel_subscriber = self.create_subscription(Twist, "/cmd_vel", self.translate_vel, 10)

    # translate Twist message from cmd_vel to Float64 messages to /vl and /vr
    def translate_vel(self, msg):
        lin_vel = msg.linear.x
        ang_vel = msg.angular.z

        wheel_dist = self.robot['wheels']["distance"]

        vel_r_msg = Float64()
        vel_l_msg = Float64()
        
        # calculate left and right velocities and store in messages
        vel_r_msg.data = lin_vel + wheel_dist * ang_vel / 2
        vel_l_msg.data = lin_vel - wheel_dist * ang_vel / 2

        self.vl_publisher.publish(vel_l_msg)
        self.vr_publisher.publish(vel_r_msg)


def main():
    rclpy.init()
    my_node = VelocityTranslator()
    rclpy.spin(my_node)

    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__init__":
    main()