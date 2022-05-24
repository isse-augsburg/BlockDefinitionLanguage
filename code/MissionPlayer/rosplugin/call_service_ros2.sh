#!/usr/bin/env bash
source /opt/ros/dashing/setup.bash
source ~/ros2_ws/install/setup.bash
echo "ros2 service call "  "$@"
ros2 service call "$@"
