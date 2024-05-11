
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import TimerAction


def generate_launch_description():
    # Get the launch directory
    package_dir = get_package_share_directory('ranger_mini_v2_gazebo')
    launch_dir = os.path.join(package_dir, 'launch')

    # Names and poses of the robots
    robots = [
        {'name': 'tb3', 'x_pose': 0.0, 'y_pose': -2.5, 'z_pose': 0.01,
                           'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'launch_file_name':'turtlebot3.launch.py', 'use_namespace':'True'},
        {'name': 'ranger', 'x_pose': -1.0, 'y_pose': -0.75, 'z_pose': 0.01,
                           'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'launch_file_name':'gazebo.launch.py', 'use_namespace':'False'},
     
        ]

    # # Simulation settings
    world = LaunchConfiguration('world')
    simulator = LaunchConfiguration('simulator')
    # rviz_config_file = LaunchConfiguration('rviz_config')
    log_settings = LaunchConfiguration('log_settings', default='True')

    # # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        # default_value=os.path.join(package_dir, 'urdf', 'world_only.model'),
        #default_value=os.path.join(package_dir, 'worlds', 'map2.world'),
        # default_value=os.path.join(package_dir, 'worlds', 'willow_garage_closed.world'),
        default_value=os.path.join(package_dir, 'worlds', 'turtlebot3_house.world'),
        # default_value=os.path.join(package_dir, 'worlds', 'willow_garage.world'),
        description='Full path to world file to load')

    declare_simulator_cmd = DeclareLaunchArgument(
        'simulator',
        default_value='gazebo',
        description='The simulator to use (gazebo or gzserver)')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(package_dir, 'rviz', 'namespaced_view.rviz'),
        description='Full path to the RVIZ config file to use.')

    # Start Gazebo with plugin providing the robot spawning service
    start_gazebo_cmd = ExecuteProcess(
        cmd=[simulator, '--verbose', '-s', 'libgazebo_ros_init.so',
                                     '-s', 'libgazebo_ros_factory.so', world],
        output='screen')

    # Define commands for launching the navigation instances
    nav_instances_cmds = []
    for robot in robots:
        nav_instances_cmds.append(GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(package_dir,
                                                           'launch',
                                                           robot['launch_file_name'])),
                launch_arguments={'namespace': robot['name'],
                                  'use_namespace': robot['use_namespace'],
                                  'use_sim_time': 'True',
                                  #'rviz_config_file': os.path.join(package_dir, 'rviz', 'namespaced_view.rviz'),
                                  'use_simulator': 'False',
                                  'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                                  'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                                  'z_pose': TextSubstitution(text=str(robot['z_pose'])),
                                  'roll': TextSubstitution(text=str(robot['roll'])),
                                  'pitch': TextSubstitution(text=str(robot['pitch'])),
                                  'yaw': TextSubstitution(text=str(robot['yaw'])),
                                  'robot_name':TextSubstitution(text=robot['name']), }.items()),

            LogInfo(
                condition=IfCondition(log_settings),
                msg=['Launching ', robot['name']]),
            # LogInfo(
            #     condition=IfCondition(log_settings),
            #     msg=[robot['name'], ' rviz config file: ', rviz_config_file]),
        ], scoped=False))

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # Add the actions to start gazebo, robots and simulations
    ld.add_action(start_gazebo_cmd)

    start_delay = 0.0
    for nav_instance in nav_instances_cmds:
        # Added delay between nav stack launching due to life_cycle manager transition freezing issue
        ld.add_action(TimerAction(period=start_delay, actions=[nav_instance]))
        start_delay += 5.0

    return ld
