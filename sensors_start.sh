#!/bin/bash

gnome-terminal --tab -- bash -c "\
cd ./e2TS;\
source devel/setup.bash;\
roslaunch image_representation voxel_test.launch;\
exec bash"

echo "Time surface successfully started!"


sleep 1

gnome-terminal --tab -- bash -c "\
cd ./dv_ros_ws;\
source devel/setup.bash;\
roslaunch dv_ros_capture mono_640.launch;\
exec bash"

echo "DVS event cameras successfully started!"

sleep 1

gnome-terminal --tab -- bash -c "\
cd ./realsense_ws;\
source devel/setup.bash;\
roslaunch realsense2_camera rs_aligned_depth.launch;\
exec bash"

echo "Realsense camera successfully started!"

sleep 1

gnome-terminal --tab -- bash -c "\
cd /home/kaizhen/dvs/xsens_630_ws;\
source devel/setup.bash;\
roslaunch xsens_mti_driver xsens_mti_node.launch;\
exec bash"

echo "Xsens MTi successfully started!"

sleep 1


# gnome-terminal --tab -- bash -c "\
# cd /home/kaizhen/dvs/vrpn_cliernt_ws;\
# source devel/setup.bash;\
# roslaunch vrpn_client_ros sample.launch;\
# exec bash"

# echo "vrpn_client_ros successfully started!"

