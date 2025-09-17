#!/bin/bash

tmux split-window -v "
bash -c \" \
cd ./e2TS; \
source devel/setup.bash; \
roslaunch image_representation voxel_test.launch; \"
"

echo "Time surface successfully started!"


sleep 1

tmux split-window -h "
sudo bash -c \" \
cd ./dv_ros_ws; \
source devel/setup.bash && \
roslaunch dv_ros_capture mono_640.launch; \"
"

echo "DVS event cameras successfully started!"

sleep 1

tmux split-window -h "
bash -c \" \
cd ./realsense_ws; \
source devel/setup.bash; \
roslaunch realsense2_camera rs_aligned_depth.launch; \"
"


echo "Realsense camera successfully started!"

sleep 1

tmux split-window -h "
sudo bash -c \" \
cd ./xsens_630_ws; \
source devel/setup.bash; \
roslaunch xsens_mti_driver xsens_mti_node.launch; \"
"

echo "Xsens MTi successfully started!"

sleep 1


tmux split-window -h "
bash -c \" \
cd ./vrpn_wx/; \
source devel/setup.bash; \
roslaunch vrpn_client_ros sample.launch server:=10.1.1.198; \"
"
echo "vrpn_client_ros successfully started!"

sleep 1
# #
# tmux split-window -h "
# bash -c \" \
# cd ./vicon_ws/; \
# source devel/setup.bash; \
# roslaunch ekf nokov.launch; \"
# "
#
# echo "ekf successfully started!"
