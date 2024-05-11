#OVERVIEW

This is ros2 simulation of a swerve drive robot named ranger_mini_v2. This workspace is a partial migration of ros1 simulation a ranger_mini_v2 robot by weston robot.Spectially the four_wheel_steering_controller package,ranger_mini_v2_naviagtion package and ranger_mini_v2_description packages have been modified significantly. 
Although urdf_geometry_parser package have not being using here.


#follow these steps to setup your workspace

1.Go inside your workspace( where your src file is located)

 ex:-cd ranger_mini_v2

2.Build the packages

colcon build --symlink-install


3.run these commands one by one in order to start the simulation

To spawn the robot and the controllers into gazebo environment-ros2 launch ranger_mini_v2_gazebo gazebo.launch.py use_sim_time:=true
To start simultanious mapping and localization- ros2 launch ranger_mini_v2_navigation slam.launch.py use_sim_time:=true
To navigate the robot using rviz2-ros2 launch ranger_mini_v2_navigation nav2.launch.py use_sim_time:=true





