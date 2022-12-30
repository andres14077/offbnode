#!/usr/bin/env bash
cp -r ~/catkin_ws/src/offbnode/models/iris_gimbal ~/Firmware/Tools/sitl_gazebo/models
cp -r ~/catkin_ws/src/offbnode/worlds ~/Firmware/Tools/sitl_gazebo
cp -r ~/catkin_ws/src/offbnode/init.d/* ~/Firmware/ROMFS/px4fmu_common/init.d-posix
cd
cd .ros/
cd eeprom/
rm parameters


cd ~/Firmware
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch offbnode mundo_y_dron.launch
