#!/usr/bin/env bash
source /opt/ros/dashing/setup.bash
source ~/ros2_ws/install/setup.bash
echo "ros2 bag record" "$@"
ros2 bag record "$@"
