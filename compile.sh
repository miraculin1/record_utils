#!/bin/bash
#
cd  ./vrpn_ws
catkin_make
cd ..

cd ./e2TS
catkin_make
cd ..

cd ./dv_ros_ws/
catkin build --cmake-args -DCMAKE_C_COMPILER=gcc-10 -DCMAKE_CXX_COMPILER=g++-10
cd ..

cd ./xsens_630_ws/
catkin build
cd ..

cd ./realsense_ws/
catkin_make
cd ..
