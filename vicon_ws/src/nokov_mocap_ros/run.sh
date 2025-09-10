#!/bin/bash
ROOT_DIR="/home/nail"
WORK_DIR="${ROOT_DIR}/Desktop/test_nokv"
PASSWD="nail"

source ${ROOT_DIR}/.bashrc
source ${WORK_DIR}/devel/setup.bash

echo ${PASSWD} | sudo -S chmod 777 /dev/ttyACM0


roslaunch mavros px4.launch & sleep 6;

# run this line
roslaunch vrpn_client_ros sample.launch server:=10.1.1.198 &sleep 3;
roslaunch ekf nokov.launch & sleep 3;

roslaunch px4ctrl run_ctrl.launch & sleep 2;
#roslaunch traj_server traj_server.launch & sleep 3;
# roslaunch traj_server traj_server.launch & sleep 3;
wait;
