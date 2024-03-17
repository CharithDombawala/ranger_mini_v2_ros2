from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
   
   joy_params=os.path.join(get_package_share_directory('ranger_mini_v2_control'),'config','joystick.yaml')

   joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[joy_params],
   )
 
   teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_node",
        parameters=[joy_params],
   )
 
   return LaunchDescription([
        joy_node,
        teleop_node
    ])
