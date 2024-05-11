follow these steps to start the simulation

1.Go inside your workspace( where your src file is located)

 ex:-cd ranger_mini_v2

2.Build the packages 

colcon build --symlink-install


3.launch these files one by one

ros2 launch ranger_mini_v2_gazebo gazebo.launch.py use_sim_time:=true
ros2 launch ranger_mini_v2_navigation slam.launch.py use_sim_time:=true
ros2 launch ranger_mini_v2_navigation nav2.launch.py use_sim_time:=true





