#!/bin/bash -x
cd
sudo rm -r ~/Firmware
git clone https://github.com/PX4/Firmware.git

cd ~/Firmware
git checkout v1.9.0
DONT_RUN=1 make px4_sitl_default gazebo
cp -rv ~/catkin_ws/src/offbnode/include/gazebo_opticalflow_plugin.h ~/Firmware/Tools/sitl_gazebo/include
cp -rv ~/catkin_ws/src/offbnode/src/gazebo_gimbal_controller_plugin.cpp ~/Firmware/Tools/sitl_gazebo/src/gazebo_gimbal_controller_plugin.cpp
DONT_RUN=1 make px4_sitl_default gazebo_iris
