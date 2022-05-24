#!/bin/bash
# Automatically start in cron with
# @reboot screen -dm start_mission.sh
cd ~/git/ma-mission
git pull
MAVID=$(cat ~/mavid.txt)
cd ~/git/martinma2/code/ros2/MissionPlayer/
./ros_mission_player.py ~/git/ma-mission/$MAVID/mission.json
