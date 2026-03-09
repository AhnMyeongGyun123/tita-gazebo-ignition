cd /home/amg/tita_0309_ws/tita_ws

colcon build --symlink-install

source install/setup.bash
ros2 launch tita_bal_ign tita_pos_2_gazebo.launch.py