from launch import LaunchDescription
from launch.launch_context import LaunchContext
from launch_ros.actions import *
from launch.actions import *
from launch.substitutions import *
from launch.event_handlers import *
from launch.events import *

from project4.disc_robot import load_disc_robot

def generate_launch_description():
    # launch arguments
    bag_in_value = LaunchConfiguration('bag_in')
    bag_in_arg = DeclareLaunchArgument('bag_in', default_value="bags/input1/")

    bag_out_value = LaunchConfiguration('bag_out')
    bag_out_arg = DeclareLaunchArgument('bag_out', default_value="bags/output/")
    
    robot_value = LaunchConfiguration('robot_file')
    robot_arg = DeclareLaunchArgument('robot_file', default_value="sim_config/robot/normal.robot")

    world_value = LaunchConfiguration('world_file')
    world_arg = DeclareLaunchArgument('world_file', default_value="sim_config/world/rectangle.world")

    # play bag files
    bag_play = ExecuteProcess(cmd=['ros2', 'bag', 'play', bag_in_value])     # command for playing bag file (with argument)
    bag_record = ExecuteProcess(cmd=['ros2', 'bag', 'record', "-a", "-o", bag_out_value ])   # command for recording bag file (with argument)

    # terminate at the end of the bag playing
    event_handler = OnProcessExit(target_action=bag_play, on_exit=[EmitEvent(event=Shutdown())])
    terminate_at_end = RegisterEventHandler(event_handler)

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

    # launch 
    ld = LaunchDescription([bag_in_arg, bag_out_arg, robot_arg, world_arg, bag_play, bag_record, sim_node, robot_state_node, terminate_at_end])
    return ld