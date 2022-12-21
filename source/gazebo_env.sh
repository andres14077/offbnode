#!/bin/bash -x
#
pushd ~/offbnode
export GAZEBO_MASTER_IP=$(docker inspect --format='{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' $(docker-compose ps -q ros_melodic_px4))
export GAZEBO_MASTER_URI=$(echo $GAZEBO_MASTER_IP):11345
popd