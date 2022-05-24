#!/usr/bin/env bash
source /opt/ros/dashing/setup.bash

numtopics=$(ros2 topic list |  awk "/$1/{print}" | wc -l)
while [ $numtopics -le 0 ]
do
	echo no topic found matching $1 
	numtopics=$(ros2 topic list |  awk "/$1/{print}" | wc -l)
done
