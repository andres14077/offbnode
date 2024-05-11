#!/bin/bash -x
if [[ -n $(ls ~/catkin_ws/src/offbnode 2>/dev/null) ]]; then
	dic=~/catkin_ws/src/offbnode
elif [[ -n $(ls ~/offbnode 2>/dev/null) ]]; then
	dic=~/offbnode
else
	echo "carpeta offbnode no encontrada"
	exit 1
fi
##copiar achivos
cp -rv $dic/init.d/1998_hitl_iris_gimbal ~/Firmware/ROMFS/px4fmu_common/init.d-posix/1999_iris_gimbal
cp -rv $dic/init.d/1999_iris_gimbal.post ~/Firmware/ROMFS/px4fmu_common/init.d-posix/1999_iris_gimbal.post
cp -rv $dic/init.d/1998_hitl_iris_gimbal ~/Firmware/ROMFS/px4fmu_common/init.d/airframes/1999_iris_gimbal
cp -rv $dic/models/hitl_iris_gimbal/* ~/Firmware/Tools/sitl_gazebo/models/iris_gimbal