
Go to this link,download the ranger_base.dae file and add it to ranger_mini_v2_description/meshes/components path

https://drive.google.com/file/d/1_UuPcv1ym_S12IVnqwNedrEESKhfejYF/view?usp=sharing


-----Build-----

colcon build --symlink-install


-----launch------

ros2 launch ranger_mini_v2_gazebo gazebo.launch.py use_sim_time:=true




-----------------------
ros2 launch ranger_mini_v2_navigation slam.launch.py use_sim_time:=true
ros2 launch ranger_mini_v2_navigation nav2.launch.py use_sim_time:=true
ros2 launch ranger_mini_v2_navigation localization.launch.py use_sim_time:=true


