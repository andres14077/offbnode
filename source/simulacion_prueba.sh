#!/usr/bin/env bash
##copiar achivos
cp -r ~/catkin_ws/src/offbnode/models/iris_gimbal ~/Firmware/Tools/sitl_gazebo/models
cp -r ~/catkin_ws/src/offbnode/worlds ~/Firmware/Tools/sitl_gazebo
cp -r ~/catkin_ws/src/offbnode/init.d/* ~/Firmware/ROMFS/px4fmu_common/init.d-posix
cd
cd .ros/
cd eeprom/
rm parameters


cd ~/Firmware
export PX4_SIM_SPEED_FACTOR=8
DONT_RUN=1 make px4_sitl_default gazebo_iris
#////////
#para solo lanzar sim
source ~/catkin_ws/devel/setup.bash    # (optional)
source ~/Firmware/Tools/setup_gazebo.bash ~/Firmware ~/Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Firmware/Tools/sitl_gazebo
roslaunch offbnode pruebas_sin_gui.launch
#reset
