#!/usr/bin/env bash
source /opt/ros/melodic/setup.bash
source /opt/ros/dashing/setup.bash
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge "$@"
