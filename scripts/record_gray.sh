rosbag record -O ./bags/$1.bag \
    /dvxplorer_left/events \
    /dvxplorer_left/triggers \
    /imu/time_ref /imu/status /imu/data \
    /camera/color/image_gray \
    /vrpn_client_node/tracker_wx1/pose \
    --tcpnodelay


rosbag info ./bags/$1.bag
