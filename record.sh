#!/bin/bash
cd /home/kaizhen/dvs/rosbag
rosbag record /dvxplorer_left/events \
    /dvxplorer_left/triggers \
    /imu/time_ref /imu/status /imu/data \
    /camera/color/image_raw /camera/depth/image_rect_raw \
    /vrpn_client_node/drone1_car/pose \
    --tcpnodelay
