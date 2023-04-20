# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud, LaserScan

import numpy as np

from project4.disc_robot import load_disc_robot
from project4.convert_map import get_occupancy_grid, vectorize


class FixedFrameBroadcaster(Node):

    def __init__(self):
        super().__init__('lidar')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_file', "STRING"),
                ('world_file', "STRING")
            ]
        )

        self.laser_publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.laser_subscriber = self.create_subscription(LaserScan, '/scan', self.publish_points, 10)
        self.pc_publisher = self.create_publisher(PointCloud, '/pc', 10)
        self.timer = self.create_timer(0.1, self.generate_lidar)

    def generate_lidar(self):
        self.robot = load_disc_robot(self.get_parameter('robot_file').value)
        self.world, world_string = get_occupancy_grid(self.get_parameter('world_file').value)


def main():
    rclpy.init()
    node = FixedFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
