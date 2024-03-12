from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.actions import Node
import xacro

ARGUMENTS = [
    DeclareLaunchArgument("model", default_value=os.path.join(get_package_share_directory('ranger_mini_v2_gazebo'),'xacro',"ranger_mini_gazebo_b.xacro")),
    DeclareLaunchArgument("paused", default_value="false"),
    DeclareLaunchArgument("use_sim_time", default_value="true"),
    DeclareLaunchArgument("gui", default_value="true"),
    DeclareLaunchArgument("headless", default_value="false"),
    DeclareLaunchArgument("debug", default_value="false"),
    DeclareLaunchArgument("joy_dev0", default_value="/dev/input/js0"),
]

def generate_launch_description():

    gazebo_ros_launch = os.path.join(get_package_share_directory('gazebo_ros'),'launch')
    urdf_path =os.path.join(get_package_share_directory('ranger_mini_v2_description'))

    xacro_file = os.path.join(urdf_path, 'urdf','ranger_mini_v2.urdf')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)  
    params = {'robot_description': doc.toxml()}
    
    ranger_mini_control_launch = os.path.join(get_package_share_directory('ranger_mini_v2_control'),'launch/control.launch.py')
    
    Include_gazebo_ros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_ros_launch,'/gazebo.launch.py']),
        launch_arguments={
            "debug": LaunchConfiguration("debug"),
            "gui": LaunchConfiguration("gui"),
            "paused": LaunchConfiguration("paused"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "headless": LaunchConfiguration("headless"),
        }.items(),
    )

    Include_ranger_mini_control=IncludeLaunchDescription(PythonLaunchDescriptionSource([ranger_mini_control_launch]),)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(
       package='gazebo_ros', 
       executable='spawn_entity.py',
       arguments=['-topic', 'robot_description',
                 '-entity', 'ranger_mini_v2'],
       output='screen'
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(Include_gazebo_ros)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)
    ld.add_action(Include_ranger_mini_control) 
    return ld

