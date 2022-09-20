#!/bin/bash
#
cd
mkdir catkin_ws/ 
mkdir catkin_ws/src
sudo apt --fix-broken install -y
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-melodic-desktop


sudo apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
## Instalacion de librerias sobre las que open -cv depnede 
sudo apt-get install -y libglew-dev libtiff5-dev zlib1g-dev libjpeg-dev libpng12-dev libjasper-dev libavcodec-dev libavformat-dev libavutil-dev libpostproc-dev libswscale-dev libeigen3-dev libtbb-dev libgtk2.0-dev pkg-config 

sudo apt-get install -y python-dev python-numpy python-py python-pytest 

sudo apt-get install -y python3-dev python3-numpy python3-py python3-pytest python-jinja2 	

sudo apt-get install -y git
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
cd ~/catkin_ws/
sudo rosdep init
rosdep update
catkin_make

echo "export SVGA_VGPU10=0" >> ~/.bashrc


echo "source /home/andres1407/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "export ROS_WORKSPACE=/home/andres/catkin_ws" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$ROS_WORKSPACE" >> ~/.bashrc
echo "export ROSCONSOLE_FORMAT='[${severity}] [${time}]:${message}'" >> ~/.bashrc

catkin_make
catkin_make

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
sudo apt-get install -y ros-melodic-control* ros-melodic-transmission-interface ros-melodic-joint-limits-interface ros-melodic-mav*
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b melodic-devel
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

# clonar paquete image_common
cd ~/catkin_ws/src
git clone https://github.com/ros-perception/image_common.git
rosdep update
# Verificar dependencias faltantes :
rosdep check --from-paths . --ignore-src --rosdistro  melodic
# Instalar dependencias faltantes :
rosdep install --from-paths . --ignore-src --rosdistro melodic -y
cd ~/catkin_ws/
catkin_make
catkin_make
catkin_make
# clonar el paquete image_geometry de vision_opencv . No se clona el repositorio 
# directamente en el workspace , pues ya se tiene el paquete de cv_bridge que
# compila con soporte para GPU.

mkdir ~/Documents
cd ~/Documents
git clone https://github.com/ros-perception/vision_opencv.git vision_opencv
cp -r ~/Documents/vision_opencv/image_geometry ~/catkin_ws/src/image_geometry
sudo apt-get update
cd ~/catkin_ws/src
git clone https://github.com/ros-visualization/rqt_image_view.git
git clone https://github.com/ros-visualization/rqt_common_plugins.git
rosdep update
# Verificar dependencias faltantes :
rosdep check --from-paths . --ignore-src --rosdistro melodic
# Instalar dependencias faltantes :
rosdep install --from-paths . --ignore-src --rosdistro melodic -y
cd ~/catkin_ws/
catkin_make
catkin_make
catkin_make
#Instalacion Geographic-Lib y mavros
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

sudo apt-get install -y ros-melodic-mav*
cd ~/catkin_ws/
catkin_make
catkin_make
catkin_make


#Firmware 

cd 
git clone https://github.com/PX4/Firmware.git
cd ~/Firmware
git checkout v1.9.0

DONT_RUN=1 make px4_sitl_default gazebo 

mv ~/offbnode ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
catkin_make
catkin_make