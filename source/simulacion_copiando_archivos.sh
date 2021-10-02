#!/usr/bin/env bash
##copiar achivos
cp -r ~/catkin_ws/src/offbnode/models/* ~/Firmware/Tools/sitl_gazebo/models
cp -r ~/catkin_ws/src/offbnode/worlds ~/Firmware/Tools/sitl_gazebo
#////////
#para solo lanzar sim
cd ~/Firmware
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch offbnode mundo_y_dron.launch
