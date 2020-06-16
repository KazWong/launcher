#! /bin/bash

export ROS_MASTER_URI=http://192.168.12.98:11311/
export ROS_IP=192.168.12.98
source ~/Codes/robot_learning/devel/setup.bash
sudo modprobe -r mttcan;sudo bash ~/Codes/robot_learning/src/drive/can_start.sh
