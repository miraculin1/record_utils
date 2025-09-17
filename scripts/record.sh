rosbag record -O ./bags/$1.bag \
    /camera/depth/image_rect_raw \
    /camera/color/image_raw \
    /dvxplorer_left/events \
    /dvxplorer_left/triggers \
    /imu/time_ref /imu/status /imu/data \
    /vrpn_client_node/Tracker091504/pose \
    --tcpnodelay


rosbag info ./bags/$1.bag
