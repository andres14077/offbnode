#!/usr/bin/env bash
cd
sudo rm -r ~/Firmware
git clone https://github.com/PX4/Firmware.git

cd ~/Firmware
git checkout v1.9.0
DONT_RUN=1 make px4_sitl_default gazebo 
cp -r ~/catkin_ws/src/offbnode/include/gazebo_opticalflow_plugin.h ~/Firmware/Tools/sitl_gazebo/include
DONT_RUN=1 make px4_sitl_default gazebo_iris
