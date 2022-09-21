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
wget -c --no-check-certificate 'https://drive.google.com/uc?export=download&id=1x_1SKbk1qCJp0v7YT7D5XzSQGnussz4i&confirm=t' -O modelos.tar.gz
tar -xzvf modelos.tar.gz
rm -f modelos.tar.gz