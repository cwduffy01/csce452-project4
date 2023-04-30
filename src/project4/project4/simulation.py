import rclpy
from rclpy.node import Node

import numpy as np
import math
import time

from std_msgs.msg import Float64, Header
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud, LaserScan
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
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


# convert LaserScan to PointCloud
def laser_scan_to_point_cloud(scan):
    return PointCloud(header=scan.header, points = [ Point32(x=scan.ranges[i]*np.cos(scan.angle_min+i*scan.angle_increment), y=scan.ranges[i]*np.sin(scan.angle_min+i*scan.angle_increment), z=0.0) for i in range(len(scan.ranges)) ])


# Node that runs the simulation
class Simulation(Node):

    move_timer = 0.1
    no_move_instruction = 0     # counter for the amount of move instructions
    vl_err = 1
    vr_err = 1
    vl = 0
    vr = 0

    def __init__(self):

        super().__init__('Simulation')

        # pull parameters from command line 
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_file', "STRING"),
                ('world_file', "STRING")
            ]
        )

        # we could pass the robot_file parameter to robot_state_publisher node to do the urdf correctly

        # load data
        self.robot = load_disc_robot(self.get_parameter('robot_file').value)
        self.world, world_string = get_occupancy_grid(self.get_parameter('world_file').value)
        self.obstacle_lines = vectorize(world_string, self.world['resolution'])

        # input topics
        self.vl_sub = self.create_subscription(Float64, '/vl', self.set_vl, 10)
        self.vr_sub = self.create_subscription(Float64, '/vr', self.set_vr, 10)

        # output topics 
        self.occupancy_pub = self.create_publisher(OccupancyGrid, '/map', 10)   # topic for map
        self.laser_pub = self.create_publisher(LaserScan, '/scan', 10)          # topic for laser scan
        self.pc_pub = self.create_publisher(PointCloud, '/pc', 10)              # topic for point cloud

        # timed actions
        self.robot_timer = self.create_timer(self.move_timer, self.move_robot)  # move robot
        self.error_timer = self.create_timer(self.robot['wheels']['error_update_rate'], self.update_errors)     # update wheel errors
        self.lidar_timer = self.create_timer(self.robot["laser"]["rate"], self.generate_lidar)  # lidar point generated 

        # set intial pose
        self.x = self.world['initial_pose'][0]
        self.y = self.world['initial_pose'][1]
        self.theta = self.world['initial_pose'][2]

        # set initial error
        self.vr_err = np.random.normal(1, math.sqrt(self.robot['wheels']['error_variance_left']))
        self.vl_err = np.random.normal(1, math.sqrt(self.robot['wheels']['error_variance_right']))

        # transform variables
        self.world_base_broadcast = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


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


    # left velocity listener
    def set_vl(self, msg):
        self.vl = msg.data
        self.no_move_instruction = 0

    # right velocity listener
    def set_vr(self, msg):
        self.vr = msg.data
        self.no_move_instruction = 0

    # error update listener
    def update_errors(self):
        self.vr_err = np.random.normal(1, math.sqrt(self.robot['wheels']['error_variance_left']))
        self.vl_err = np.random.normal(1, math.sqrt(self.robot['wheels']['error_variance_right']))

    # robot motion listener
    def move_robot(self):
        # if no instruction for 1 second, dont move
        if(self.no_move_instruction >= 1/self.move_timer):
            return

        # account for error
        vl = self.vl * self.vl_err
        vr = self.vr * self.vr_err

        # compute new state
        l = self.robot['wheels']['distance'] # distance between wheels

        # handle three potential motions
        if(vl == vr):   # go straight
            x = self.x + math.cos(self.theta) * vl * self.move_timer
            y = self.y + math.sin(self.theta) * vl * self.move_timer
            new_state = [x, y, self.theta]

        elif(vl == vr * -1):    # turn in place
            w = (vr - vl) / l
            theta = self.theta + w * self.move_timer
            new_state = [self.x, self.y, theta]

        else:   # rotation 
            R = l/2 * (vr + vl) / (vr - vl) # distance to ICC
            w = (vr - vl) / l # angular velocity around ICC
            c = [self.x - R * math.sin(self.theta), self.y + R * math.cos(self.theta)] # location of ICC
            m1 = np.matrix([[math.cos(w*self.move_timer), -1 * math.sin(w*self.move_timer), 0],
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

        # collision detection
        for line in self.obstacle_lines:
            dist = point_line_distance(line[0], line[1], line[2], line[3], new_state[0], new_state[1])
            if dist < self.robot['body']['radius']:
                return

        # set the state to the new state
        self.x = new_state[0]
        self.y = new_state[1]
        self.theta = new_state[2]

        # publish transform message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'

        # set translation of transform
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # generate quaternion from the euler angles of the robot
        quat = quaternion_from_euler(0, 0, self.theta)

        # set rotation of transform to quaternion values
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # send transform
        self.world_base_broadcast.sendTransform(t)

        # count movement instruction
        self.no_move_instruction += 1

    # generate lidar points listener
    def generate_lidar(self): 
        robot_laser = self.robot["laser"]
        num_scans = robot_laser["count"]

        # create laser scan message
        ls = LaserScan()
        ls.header.frame_id = "laser" 
        ls.angle_min = robot_laser["angle_min"]
        ls.angle_max = robot_laser["angle_max"]
        ls.angle_increment = (ls.angle_max - ls.angle_min) / num_scans
        ls.scan_time = float(robot_laser["rate"]) 
        ls.range_min = robot_laser["range_min"]
        ls.range_max = robot_laser["range_max"]
        ls.ranges = []
        ls.intensities = np.zeros(num_scans, dtype=float).tolist()

        # get laser transform and times
        trans_time = rclpy.time.Time()
        start_time = time.time()
        laser_to_world_tf = self.tf_buffer.lookup_transform(
            "world",
            "laser",
            trans_time)
        ls.header.stamp = trans_time.to_msg()

        # get translation and rotation of laser frame
        laser_x = laser_to_world_tf.transform.translation.x
        laser_y = laser_to_world_tf.transform.translation.y
        q = laser_to_world_tf.transform.rotation
        laser_theta = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))

        # for each angle, get point
        curr_angle = laser_theta + ls.angle_min
        for i in range(num_scans): 
            # account for failing scan
            rand = np.random.rand()
            if rand < self.robot['laser']['fail_probability']:
                ls.ranges.append(float('nan'))
                curr_angle += ls.angle_increment
                continue

            # get minimum distance for each scan 
            min_dist = float("inf")
            for seg in self.obstacle_lines:     # get point for each segment
                intersect = line_ray_intersection(laser_x, laser_y, curr_angle, *seg)   # get distance to line segment
                if intersect > ls.range_min**2 and intersect < ls.range_max**2:         # check within range
                    if intersect < min_dist**2 or min_dist == float("inf"):             # check if it is lower than 
                        min_dist = intersect

            # add scan error
            err = np.random.normal(0, math.sqrt(self.robot['laser']['error_variance']))
            ls.ranges.append(math.sqrt(min_dist) + err)

            # next angle
            curr_angle += ls.angle_increment

        # set time increment of scan
        end_time = time.time()
        time_change = (end_time - start_time) / 1000 + float(robot_laser["rate"])
        ls.time_increment = time_change

        # publish point cloud points
        pc = laser_scan_to_point_cloud(ls)
        self.pc_pub.publish(pc)
        self.laser_pub.publish(ls)
        

def main():
    rclpy.init()
    sim = Simulation()
    rclpy.spin(sim)

    sim.destroy_node()
    rclpy.shutdown()

if __name__ == "__init__":
    main()