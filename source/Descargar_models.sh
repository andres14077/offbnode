#!/bin/bash -x
if [[ -n $(ls ~/catkin_ws/src/offbnode 2>/dev/null) ]]; then
	dic=~/catkin_ws/src/offbnode
elif [[ -n $(ls ~/offbnode 2>/dev/null) ]]; then
	dic=~/offbnode
else
	echo "carpeta offbnode no encontrada"
	exit 1
fi
cd $dic
wget -c --no-check-certificate 'https://drive.google.com/uc?export=download&id=1Jek71VO0AhEsq4dGM9NalkFbJyFQfCmB&confirm=t' -O textura.tar.gz
tar -xzvf textura.tar.gz
rm -f textura.tar.gz