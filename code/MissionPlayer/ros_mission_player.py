#!/usr/bin/env python3
import mission_player as mp
import sys
import rosplugin
import syncplugin
import os

def main():
    # Add ROS-Specific Functionality
    mp.add_plugin(rosplugin.blocks)
    mp.add_plugin(syncplugin.blocks)

    home = home = os.getenv("HOME")

    # Play Mission
    if len(sys.argv) == 2:
        mp.play_mission(sys.argv[1])
    else:

        mp.play_mission("%s/git/ma-mission/%d/mission.json"%(home, mp.get_mavid()))


if __name__ == "__main__":
    main()
