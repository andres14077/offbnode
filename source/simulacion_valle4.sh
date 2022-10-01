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
DONT_RUN=1 make px4_sitl_default gazebo_iris
#////////
#para solo lanzar sim
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
if [[ -z $1 ]]; then
	angulo_entrada=0
else
	angulo_entrada=$1
fi
if [[ -z $2 ]]; then
	carpeta=v4/
else
	carpeta=$2
fi
roslaunch offbnode valle4.launch angulo_entrada:=$angulo_entrada carpeta_imagenes:=$carpeta
#reset
