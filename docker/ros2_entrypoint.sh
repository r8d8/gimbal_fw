#!/bin/bash

# Build ROS dependency
source /opt/ros/${ROS_DISTRO}/setup.bash

# Restart udev daemon
sudo service udev restart

SESSION_NAME="comp_ros2"
tmux new-session -ds $SESSION_NAME
tmux send-keys -t $SESSION_NAME:0.0 "ros2 launch mavros apm.launch fcu_url:=udp://@192.168.1.89:14551 gcs_url:=udp://@192.168.191.18:14550" C-m
tmux split-window  -h -p 50 -t $SESSION_NAME:0
tmux send-keys -t $SESSION_NAME:0.1 "ros2 node list" C-m
tmux attach -t $SESSION_NAME

$@