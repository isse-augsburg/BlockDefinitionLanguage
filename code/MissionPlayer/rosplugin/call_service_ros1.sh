#!/usr/bin/env bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosservice call "$@"
