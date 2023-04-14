import rclpy
from rclpy.node import Node

import numpy as np
import math

from std_msgs.msg import Float64
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

from project4.disc_robot import load_disc_robot
from project4.convert_map import 

class Simulation(Node):

    def __init__(self):
        self.robot = load_disc_robot('sim_config/robot/normal.robot')
        
        super().__init__('Simulation')
        # inputs
        # might have to change refresh time for subs / pubs
        self.vl_sub = self.create_subscription(Float64, '/vl', self.set_vl, 10)
        self.vr_sub = self.create_subscription(Float64, '/vr', self.set_vr, 10)

        # outputs
        self.occupancy_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.laser_pub = self.create_publisher(LaserScan, '/scan', 10)

    def set_vl(self, msg):
        self.vl = msg

    def set_vr(self, msg):
        self.vr = msg

def main():
    rclpy.init()
    sim = Simulation()
    rclpy.spin(sim)

    sim.destroy_node()
    rclpy.shutdown()

if __name__ == "__init__":
    main()


        
        