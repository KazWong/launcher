#!/bin/bash

ip=(`ifconfig | grep 'wlan0\|wlo1' -A2`)
export ROS_IP=${ip[5]}
if [ -n "$1" ]; then
        export ROS_MASTER_URI=http://"$1":11311/
else
        export ROS_MASTER_URI=http://${ip[5]}:11311/
fi
