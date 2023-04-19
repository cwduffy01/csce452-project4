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

# convert roll pitch and yaw to a quaternion
def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class Simulation(Node):

    move_timer = 0.1
    no_move_instruction = 0
    vl = 0
    vr = 0

    def __init__(self):

        super().__init__('Simulation')
        # inputs
        self.vl_sub = self.create_subscription(Float64, '/vl', self.set_vl, 10)
        self.vr_sub = self.create_subscription(Float64, '/vr', self.set_vr, 10)

        # outputs
        self.occupancy_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.laser_pub = self.create_publisher(LaserScan, '/scan', 10)

        # load data
        self.robot = load_disc_robot('sim_config/robot/normal.robot')
        self.world = get_occupancy_grid('sim_config/world/pillars.world')
        self.x = self.world['initial_pose'][0]
        self.y = self.world['initial_pose'][1]
        self.theta = self.world['initial_pose'][2]


        # transforms
        self.world_base_broadcast = TransformBroadcaster(self) # world -> base
        # base -> laser in launch file?

        # robot movement (may need to change delay)
        self.robot_timer = self.create_timer(self.move_timer, self.move_robot)
        self.lidar_timer = self.create_timer(0.1, self.lidar_frame_callback)

        # publish occupancy grid
        header = Header()
        header.frame_id = 'world'

        metadata = MapMetaData()
        metadata.resolution = self.world['resolution']
        metadata.width = self.world['width']
        metadata.height = self.world['height']

        og = OccupancyGrid()
        og.header = header
        og.info = metadata
        og.data = self.world['map']

        self.occupancy_pub.publish(og)
        # print(og)

    def set_vl(self, msg):
        self.vl = msg
        self.no_move_instruction = 0

    def set_vr(self, msg):
        self.vr = msg
        self.no_move_instruction = 0

    def move_robot(self):
        # if no instruction for 1 second, dont move
        # if(self.no_move_instruction >= 1/self.move_timer):
        #     return

        self.vl = 0.1
        self.vr = 0.2
        # compute new state
        print(f'current state: ({self.x}, {self.y}, {self.theta})')
        l = self.robot['wheels']['distance'] # distance between wheels

        if(self.vl == self.vr): # go straight
            print('straight')
            x = self.x + math.cos(self.theta) * self.vl * self.move_timer
            y = self.y + math.sin(self.theta) * self.vl * self.move_timer
            new_state = [x, y, self.theta]

        elif(self.vl == self.vr * -1): # turn in place
            print('turn')
            w = (self.vr - self.vl) / l
            theta = self.theta + w * self.move_timer
            new_state = [self.x, self.y, theta]

        else:
            print('big')
            R = l/2 * (self.vr + self.vl) / (self.vr - self.vl) # distance to ICC
            w = (self.vr - self.vl) / l # angular velocity around ICC
            c = [self.x - R * math.sin(self.theta), self.y + R * math.cos(self.theta)] # location of ICC
            m1 = np.matrix([
                [math.cos(w*self.move_timer), -1 * math.sin(w*self.move_timer), 0],
                [math.sin(w*self.move_timer), math.cos(w*self.move_timer), 0],
                [0, 0, 1]])
            v1 = np.matrix([[self.x - c[0]],
                            [self.y - c[1]], 
                            [self.theta]])
            v2 = np.matrix([[c[0]], 
                            [c[1]], 
                            [w*self.move_timer]])
            new_state = m1 * v1 + v2
            new_state = new_state.flatten().tolist()[0]
        
        print(f'new state: ({new_state[0]}, {new_state[1]}, {new_state[2]})')

        # do collision detection
        self.x = new_state[0]
        self.y = new_state[1]
        self.theta = new_state[2]

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        quat = quaternion_from_euler(0, 0, self.theta)

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.world_base_broadcast.sendTransform(t)

        self.no_move_instruction += 1

    def lidar_frame_callback(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base'
        t.child_frame_id = 'lidar'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.world_base_broadcast.sendTransform(t)

def main():
    rclpy.init()
    sim = Simulation()
    rclpy.spin(sim)

    sim.destroy_node()
    rclpy.shutdown()

if __name__ == "__init__":
    main()


        
        