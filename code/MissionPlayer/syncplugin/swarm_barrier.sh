#!/usr/bin/env bash
source /opt/ros/dashing/setup.bash
source ~/ros2_ws/install/setup.bash
# Echo the topic and quit when we get a message
ros2 run swarm_sync barrier_node "$@"
