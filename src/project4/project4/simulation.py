import rclpy
from rclpy.node import Node

import numpy as np
import math

from std_msgs.msg import Float64, Header
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Point32, Pose
from sensor_msgs.msg import PointCloud, LaserScan
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from project4.disc_robot import load_disc_robot
from project4.convert_map import get_occupancy_grid, vectorize
from project4.line_intersection import line_ray_intersection, point_line_distance


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


def laser_scan_to_point_cloud(scan):
    return PointCloud(header=scan.header, points = [ Point32(x=scan.ranges[i]*np.cos(scan.angle_min+i*scan.angle_increment), y=scan.ranges[i]*np.sin(scan.angle_min+i*scan.angle_increment), z=0.0) for i in range(len(scan.ranges)) ])


class Simulation(Node):

    move_timer = 0.1
    no_move_instruction = 0
    vl = 0
    vr = 0

    def __init__(self):

        super().__init__('Simulation')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_file', "STRING"),
                ('world_file', "STRING")
            ]
        )

        # load data
        self.robot = load_disc_robot(self.get_parameter('robot_file').value)
        self.world, world_string = get_occupancy_grid(self.get_parameter('world_file').value)
        self.obstacle_lines = vectorize(world_string, self.world['resolution'])

        # inputs
        self.vl_sub = self.create_subscription(Float64, '/vl', self.set_vl, 10)
        self.vr_sub = self.create_subscription(Float64, '/vr', self.set_vr, 10)

        # outputs
        self.occupancy_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.laser_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.publish_points, 10)
        self.pc_pub = self.create_publisher(PointCloud, '/pc', 10)
        self.timer = self.create_timer(self.robot["laser"]["rate"], self.generate_lidar)

        print(self.obstacle_lines)

        # set intial pose
        self.x = self.world['initial_pose'][0]
        self.y = self.world['initial_pose'][1]
        self.theta = self.world['initial_pose'][2]


        # transforms
        self.world_base_broadcast = TransformBroadcaster(self) # world -> base
        # base -> laser in launch file?

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # robot movement (may need to change delay)
        self.robot_timer = self.create_timer(self.move_timer, self.move_robot)

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
        self.vl = msg.data
        self.no_move_instruction = 0

    def set_vr(self, msg):
        self.vr = msg.data
        self.no_move_instruction = 0

    def move_robot(self):
        # if no instruction for 1 second, dont move
        # if(self.no_move_instruction >= 1/self.move_timer):
        #     return

        # self.vl = 0.4
        # self.vr = 0.3
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
        for line in self.obstacle_lines:
            dist = point_line_distance(line[0], line[1], line[2], line[3], new_state[0], new_state[1])
            if dist < self.robot['body']['radius']:
                print('crash')
                return

        print(new_state)
        self.x = new_state[0]
        self.y = new_state[1]
        self.theta = new_state[2]

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'

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

    def generate_lidar(self): 
        ls = LaserScan()

        robot_laser = self.robot["laser"]

        ls.header.frame_id = "laser" 

        num_scans = robot_laser["count"]

        ls.angle_min = robot_laser["angle_min"]
        ls.angle_max = robot_laser["angle_max"]
        ls.angle_increment = (ls.angle_max - ls.angle_min) / num_scans
        ls.time_increment = robot_laser["rate"] / num_scans
        ls.scan_time = 0.0

        ls.range_min = robot_laser["range_min"]
        ls.range_max = robot_laser["range_max"]

        # CHANGE TO PUT POINTS ON MAP
        ls.ranges = []
        ls.intensities = np.zeros(num_scans, dtype=float).tolist()

        trans_time = rclpy.time.Time()

        t = self.tf_buffer.lookup_transform(
            "laser",
            "world",
            rclpy.time.Time())
        
        laser_trans = np.array([[t.transform.translation.x], [t.transform.translation.y]])
        q = t.transform.rotation
        laser_theta = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))

        rot_mat = np.array([[np.cos(laser_theta), -np.sin(laser_theta)],
                            [np.sin(laser_theta), np.cos(laser_theta)]])

        curr_angle = ls.angle_min
        for i in range(num_scans): 
            min_dist = float("inf")   # maybe change
            for seg in self.obstacle_lines:
                # laser_seg = seg

                new_start = rot_mat @ np.array([[seg[0]], [seg[1]]]) + laser_trans
                new_end =   rot_mat @ np.array([[seg[2]], [seg[3]]]) + laser_trans

                laser_seg = (new_start[0][0], new_start[1][0], new_end[0][0], new_end[1][0])

                # self.get_logger().info(f"SEG:       {seg}")
                # self.get_logger().info(f"LASER_SEG: {laser_seg}")

                intersect = line_ray_intersection(0, 0, curr_angle, *laser_seg)
                # self.get_logger().info("intersect distance: %d" % (intersect))
                if intersect > ls.range_min and intersect < ls.range_max:
                    if intersect < min_dist or min_dist == float("inf"):
                        min_dist = intersect
            ls.ranges.append(min_dist)
            curr_angle += ls.angle_increment

        
        ls.header.stamp = trans_time.to_msg()

        self.laser_pub.publish(ls)
        pc = laser_scan_to_point_cloud(ls)
        self.pc_pub.publish(pc)


    def publish_points(self, msg):
        # pc = laser_scan_to_point_cloud(msg)
        # self.pc_pub.publish(pc)
        pass

def main():
    rclpy.init()
    sim = Simulation()
    rclpy.spin(sim)

    sim.destroy_node()
    rclpy.shutdown()

if __name__ == "__init__":
    main()
                