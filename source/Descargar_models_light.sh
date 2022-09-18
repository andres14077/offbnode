#!/usr/bin/env bash
if [[ -n $(ls ~/catkin_ws/src/offbnode 2>/dev/null) ]]; then
	dic=~/catkin_ws/src/offbnode
elif [[ -n $(ls ~/offbnode 2>/dev/null) ]]; then
	dic=~/offbnode
else
	echo "carpeta offbnode no encontrada"
	exit 1
fi
cd $dic
wget -c --no-check-certificate 'https://drive.google.com/uc?export=download&id=1S4KLVSew-6n2qKmZMl_saBsk_0gFZxbJ&confirm=t' -O modelos.tar.gz
tar -xzvf modelos.tar.gz
rm -f modelos.tar.gz
exit 0