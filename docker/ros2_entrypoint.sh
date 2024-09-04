#!/bin/bash
# Basic entrypoint for ROS / Colcon Docker containers
 
# Build ROS dependency
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
source /opt/ros/${ROS_DISTRO}/setup.bash

# Restart udev daemon
sudo service udev restart

$@