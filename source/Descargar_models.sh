#!/usr/bin/env bash
cd ~/catkin_ws/src/offbnode
wget -c --no-check-certificate 'https://drive.google.com/uc?export=download&id=1x_1SKbk1qCJp0v7YT7D5XzSQGnussz4i&confirm=t' -O modelos.tar.gz
tar -xzvf modelos.tar.gz
rm -f modelos.tar.gz
