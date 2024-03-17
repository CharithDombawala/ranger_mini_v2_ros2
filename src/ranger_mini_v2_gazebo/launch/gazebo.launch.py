from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription,ExecuteProcess,DeclareLaunchArgument)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
import os
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.actions import Node
import xacro

ARGUMENTS = [
    DeclareLaunchArgument("model", default_value=os.path.join(get_package_share_directory('ranger_mini_v2_gazebo'),'xacro',"ranger_mini_gazebo_b.xacro")),
    DeclareLaunchArgument("paused", default_value="false"),
    DeclareLaunchArgument("use_sim_time", default_value="true"),
    DeclareLaunchArgument("gui", default_value="true"),
    DeclareLaunchArgument("headless", default_value="False"),
    DeclareLaunchArgument("debug", default_value="false"),
    DeclareLaunchArgument("joy_dev0", default_value="/dev/input/js0"),
   # DeclareLaunchArgument('world',default_value="/home/charith-2204/ranger_mini_v2/src/ranger_mini_v2_gazebo/worlds/room_with_walls_1.sdf"),
    DeclareLaunchArgument('use_simulator',default_value='True',)

         
]

def generate_launch_description():
    launch_dir=os.path.join(get_package_share_directory('ranger_mini_v2_gazebo'),"launch")
    urdf_path =os.path.join(get_package_share_directory('ranger_mini_v2_description'))
    pkg_dir = get_package_share_directory('warehouse_robot_spawner_pkg')
    
    
    xacro_file = os.path.join(urdf_path, 'urdf','ranger_mini_v2.urdf')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)  
    params = {'robot_description': doc.toxml()}
    
    
    use_simulator = LaunchConfiguration('use_simulator')
    #world = os.path.join(pkg_dir, 'worlds', 'warehouse.world')
    world = ""
    headless = LaunchConfiguration('headless')
    
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_dir, 'models')
    
    ranger_mini_control_launch = os.path.join(get_package_share_directory('ranger_mini_v2_control'),'launch/control.launch.py')
    
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world],
        cwd=[launch_dir],
        output='screen')

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression(
            [use_simulator, ' and not ', headless])),
        cmd=['gzclient'],
        cwd=[launch_dir],
        output='screen')


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
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)
    ld.add_action(Include_ranger_mini_control) 
    return ld

