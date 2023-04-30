from launch import LaunchDescription
from launch.launch_context import LaunchContext
from launch_ros.actions import *
from launch.actions import *
from launch.substitutions import *
from launch.event_handlers import *
from launch.events import *

from project4.disc_robot import load_disc_robot

def generate_launch_description():
    bag_out_value = LaunchConfiguration('bag_out')
    bag_out_arg = DeclareLaunchArgument('bag_out', default_value="bags/output/")
    
    robot_value = LaunchConfiguration('robot_file')
    robot_arg = DeclareLaunchArgument('robot_file', default_value="sim_config/robot/normal.robot")

    world_value = LaunchConfiguration('world_file')
    world_arg = DeclareLaunchArgument('world_file', default_value="sim_config/world/rectangle.world")

    # start robot node
    robot = load_disc_robot("sim_config/robot/normal.robot")
    robot_state_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot['urdf']}],
        )

    # start simulation node
    sim_node = Node(
        package='project4', 
        executable='sim',
        parameters=[{"robot_file": robot_value}, {"world_file": world_value}]
    )

    # start simulation node
    vel_node = Node(
        package='project4', 
        executable='vel',
        parameters=[{"robot_file": robot_value}]
    )

    # start simulation node
    nav_node = Node(
        package='project4', 
        executable='nav',
        parameters=[{"robot_file": robot_value}]

    )

    # launch 
    ld = LaunchDescription([bag_out_arg, robot_arg, world_arg, sim_node, vel_node, nav_node, robot_state_node])
    return ld