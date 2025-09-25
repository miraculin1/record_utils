rosbag record -O ./bags/$1.bag \
    /dvxplorer_left/events \
    /dvxplorer_left/triggers \
    /imu/time_ref /imu/status /imu/data \
    /camera/color/image_raw \
    /vrpn_client_node/tracker_wx/pose \
    /camera/depth/image_rect_raw \
    --tcpnodelay


rosbag info ./bags/$1.bag
