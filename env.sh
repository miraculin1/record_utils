#!/bin/bash
#===event
sudo add-apt-repository ppa:inivation-ppa/inivation
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install software-properties-common build-essential -y
# sudo apt-get install dv-processing dv-runtime-dev
sudo apt-get install gcc-10 g++-10 cmake -y
sudo apt-get install libcaer-dev -y
sudo apt-get install libfmt-dev python3-catkin python3-catkin-tools ros-noetic-catkin ros-noetic-camera-info-manager -y

wget https://archives.boost.io/release/1.76.0/source/boost_1_76_0.tar.gz
tar -xzf boost_1_76_0.tar.gz
rm boost_1_76_0.tar.gz
cd boost_1_76_0
./bootstrap.sh
./b2 install --prefix="out" --build-dir="build"
cd ..

git clone https://gitlab.com/inivation/dv/dv-processing.git
cd dv-processing
git checkout 1.7.9
mkdir build
cd build
CC=gcc-10 CXX=g++-10 cmake -DBOOST_ROOT=../boost_1_76_0/out -DBOOST_INCLUDEDIR=../boost_1_76_0/out/include -DBOOST_LIBRARYDIR=../boost_1_76_0/out/lib -DCMAKE_INSTALL_PREFIX=/usr ..
make -j
sudo make install
cd ..

git clone https://gitlab.com/inivation/dv/dv-runtime.git
cd dv-runtime
git checkout 1.6.2
cmake -DBOOST_ROOT=../boost_1_76_0/out -DBOOST_INCLUDEDIR=../boost_1_76_0/out/include -DBOOST_LIBRARYDIR=../boost_1_76_0/out/lib -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_C_COMPILER=gcc-10 -DCMAKE_CXX_COMPILER=g++-10 .
make -j
sudo make install
cd ..



#===realsense
sudo apt-get install apt-transport-https
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
sudo apt-get install librealsense2-dkms -y
sudo apt-get install librealsense2-utils -y
sudo apt-get install librealsense2-dev -y
sudo apt-get install ros-noetic-ddynamic-reconfigure ros-noetic-realsense2-camera ros-noetic-librealsense2 -y

