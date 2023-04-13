from launch import LaunchDescription
from launch_ros.actions import *
from launch.actions import *
from launch.substitutions import *
from launch.event_handlers import *
from launch.events import *

def generate_launch_description():

    ## PROJECT 3 LAUNCH FILE CODE ##

    """
    # initialize command line arguments
    bag_in_value = LaunchConfiguration('bag_in')
    bag_in_arg = DeclareLaunchArgument('bag_in', default_value="bags/example1/")

    bag_out_value = LaunchConfiguration('bag_out')
    bag_out_arg = DeclareLaunchArgument('bag_out', default_value="output/")

    bag_play = ExecuteProcess(cmd=['ros2', 'bag', 'play', bag_in_value])     # command for playing bag file (with argument)
    bag_record = ExecuteProcess(cmd=['ros2', 'bag', 'record', "-a", "-o", bag_out_value ])   # command for recording bag file (with argument)
    track_node = Node(package='project3', executable='track')   # node for tracking people
    people_node = Node(package='project3', executable='people') # node for identifying people

    # handle terminating
    event_handler = OnProcessExit(target_action=bag_play, on_exit=[EmitEvent(event=Shutdown())])
    terminate_at_end = RegisterEventHandler(event_handler)

    # generate launch description and return
    ld = LaunchDescription([ bag_in_arg, bag_out_arg, bag_play, bag_record, track_node, people_node, terminate_at_end ])
    return ld
    """