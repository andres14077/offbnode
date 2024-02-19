#!/bin/bash -x
#
cd
mkdir -p catkin_ws/src
sudo apt-get update
sudo apt-get --fix-broken install -y
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-get install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update

sudo apt-get install -y git python3-dev python3-numpy python3-py python3-pytest python3-opencv python3-pip ros-melodic-base python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential libglew-dev libtiff5-dev zlib1g-dev libjpeg-dev libavcodec-dev libavformat-dev libavutil-dev libpostproc-dev libswscale-dev libeigen3-dev libtbb-dev libgtk2.0-dev pkg-config python-dev python-numpy python-py python-pytest python-pip python-jinja2 ros-melodic-control* ros-melodic-transmission-interface ros-melodic-joint-limits-interface ros-melodic-mav*


pip install numpy toml
pip3 install --upgrade pip
pip3 install catkin_pkg rospkg tensorflow scikit-learn seaborn


# Actualizar gazebo
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get upgrade



#echo "export SVGA_VGPU10=0" >> ~/.bashrc
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "export ROS_WORKSPACE=~/catkin_ws" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:\$ROS_WORKSPACE" >> ~/.bashrc
echo "export ROSCONSOLE_FORMAT='[\${severity}] [\${time}]:\${message}'" >> ~/.bashrc
echo "export ROS_IP=\$(hostname -I| awk '{print \$1}')" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://\$(echo \$ROS_IP):11311/" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/melodic/setup.bash
cd ~/catkin_ws/
sudo rosdep init
rosdep update

catkin_make
catkin_make
catkin_make

source ~/catkin_ws/devel/setup.bash

cd
mkdir opencv3
cd opencv3

## Clonar repositorio
git clone https://github.com/opencv/opencv.git opencv
cd opencv
git checkout 3.3.1
cd ..
## OpenCV Extras
git clone https://github.com/opencv/opencv_extra.git
cd opencv_extra/
git checkout 3.3.1
cd ..
cd opencv

## Crear directorio para construir los paquetes
mkdir build
cd build/
## Configuracion de la comiplacion e instalacion
export PATH=$PATH:/usr/local/cuda/bin
export LD_LIBRARY_PATH=/usr/local/cuda-8.0/lib64
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr -D BUILD_PNG=OFF -D BUILD_TIFF=OFF -D BUILD_TBB=OFF -D BUILD_JPEG=OFF -D BUILD_JASPER=OFF -D BUILD_ZLIB=OFF -D BUILD_EXAMPLES=ON -D BUILD_JAVA=OFF -D BUILD_opencv_python2=ON -D BUILD_opencv_python3=OFF -D WITH_OPENCL=OFF -D WITH_OPENMP=OFF -D WITH_FFMPEG=ON  -DWITH_GSTREAMER=OFF -DWITH_GSTREAMER_0_10=OFF -DWITH_CUDA=ON -DWITH_GTK=ON -DWITH_VTK=OFF -DWITH_TBB=ON -DWITH_1394=OFF -DWITH_OPENEXR=OFF -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-8.0 -DCUDA_ARCH_BIN=’3.0 3.5 5.0 6.0 6.2 ’ -DCUDA_ARCH_PTX="" -DINSTALL_C_EXAMPLES=ON -DINSTALL_TESTS=OFF -DOPENCV_TEST_DATA_PATH=../opencv_extra/testdata ../opencv ..
## Comiplar librerias con siete de los procesadores de la CPU del dispositivo
make -j7
## Instalar librerias
sudo make install

cd ~/catkin_ws/src
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b melodic-devel
cd gazebo_ros_pkgs
git checkout 6ce46e3
cd ..
rosdep update
# Verificar dependencias faltantes :
rosdep check --from-paths . --ignore-src --rosdistro melodic
# Instalar dependencias faltantes :
rosdep install --from-paths . --ignore-src --rosdistro melodic -y
cd ~/catkin_ws/
catkin_make
catkin_make
catkin_make
sudo apt-get update

### Instalacion Geographic-Lib y mavros
cd
git clone git://git.code.sourceforge.net/p/geographiclib/code geographiclib
cd geographiclib
mkdir BUILD
cd BUILD
cmake ..
cmake -D CMAKE_INSTALL_PREFIX=/usr/ -D GEOGRAPHICLIB_DATA=/usr/share/GeographicLib -D GEOGRAPHICLIB_LIB_TYPE=SHARED -D CMAKE_BUILD_TYPE=Release .
make # compile the library and utilities
make test # run some tests
sudo make install
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh

###Firmware

cd
git clone https://github.com/PX4/Firmware.git
cd ~/Firmware
git checkout v1.9.0

DONT_RUN=1 make px4_sitl_default gazebo
cp ~/offbnode/src/gazebo_gimbal_controller_plugin.cpp ~/Firmware/Tools/sitl_gazebo/src/gazebo_gimbal_controller_plugin.cpp
DONT_RUN=1 make px4_sitl_default gazebo

mv ~/offbnode ~/catkin_ws/src/
cd ~/catkin_ws
# Verificar dependencias faltantes :
rosdep check --from-paths . --ignore-src --rosdistro melodic
# Instalar dependencias faltantes :
rosdep install --from-paths . --ignore-src --rosdistro melodic -y
catkin_make
catkin_make
catkin_make

cd ~/catkin_ws/src/offbnode
source source/Descargar_models.sh
source source/Copiar_modelos_y_configuraciones.sh

echo "pushd ~/Firmware" >> ~/.bashrc
echo "source Tools/setup_gazebo.bash \$(pwd) \$(pwd)/build/px4_sitl_default" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:\$(pwd)" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:\$(pwd)/Tools/sitl_gazebo" >> ~/.bashrc
echo "popd" >> ~/.bashrc
