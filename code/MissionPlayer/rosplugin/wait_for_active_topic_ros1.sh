#!/usr/bin/env bash
source /opt/ros/melodic/setup.bash
# $1 must be the topic name
# Echo the topic and quit when we get a message
rostopic echo $1 | sed -e '/---/q'
