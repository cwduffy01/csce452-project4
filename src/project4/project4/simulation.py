import rclpy
from rclpy.node import Node

import numpy as np
import math

from std_msgs.msg import Float64, Header
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from project4.disc_robot import load_disc_robot
from project4.convert_map import get_occupancy_grid

class Simulation(Node):

    def __init__(self):

        super().__init__('Simulation')
        # inputs
        # might have to change refresh time for subs / pubs
        self.vl_sub = self.create_subscription(Float64, '/vl', self.set_vl, 10)
        self.vr_sub = self.create_subscription(Float64, '/vr', self.set_vr, 10)

        # outputs
        self.occupancy_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.laser_pub = self.create_publisher(LaserScan, '/scan', 10)

        # load data
        self.robot = load_disc_robot('sim_config/robot/normal.robot')
        self.world = get_occupancy_grid('sim_config/world/pillars.world')

        # transforms
        self.world_base_broadcast = TransformBroadcaster(self) # world -> base
        # base -> laser in launch file?

        # robot movement (may need to change delay)
        self.robot_timer = self.create_timer(0.1, self.move_robot)

        # publish occupancy grid
        header = Header()
        header.frame_id = 'occupancy_grid'

        metadata = MapMetaData()
        metadata.resolution = self.world['resolution']
        metadata.width = self.world['width']
        metadata.height = self.world['height']

        og = OccupancyGrid()
        og.header = header
        og.info = metadata
        og.data = self.world['map']

        self.occupancy_pub.publish(og)
        print(og)

    def set_vl(self, msg):
        self.vl = msg

    def set_vr(self, msg):
        self.vr = msg

    def move_robot(self):
        # compute new state
        move_x = 1.0
        move_y = 1.0

        theta_x = 1.0
        theta_y = 1.0

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base'

        t.transform.translation.x = move_x
        t.transform.translation.y = move_y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = theta_x
        t.transform.rotation.y = theta_y
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.0

        self.world_base_broadcast.sendTransform(t)

def main():
    rclpy.init()
    sim = Simulation()
    rclpy.spin(sim)

    sim.destroy_node()
    rclpy.shutdown()

if __name__ == "__init__":
    main()


        
        