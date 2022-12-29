#!/bin/bash -x
#
pushd ~/offbnode
export GAZEBO_MASTER_IP=$(docker inspect --format='{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' $(docker-compose ps -q ros_melodic_px4))
#export GAZEBO_MASTER_IP=192.168.1.101
export GAZEBO_MASTER_URI=$(echo $GAZEBO_MASTER_IP):11345
popd

pushd ~/Firmware
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
popd