Block Definition Language (BDL)
==================================================================
BDL is a modular and extinsible mission planning framework for one or more agents.
It can be used to define, deploy and execute missions (tasks) to one or multiple devices.
In this repository extensions for interfacing with ROS and ROS2 are provided as well as a plugin for synchronizing multiple agents via barriers. The Framework also provides a simple way to integrate sensors and sensor data by providing a library that allows automatic integration of simple sensors into the system.


# Repository Structure
* `/documentation`:  Additional documentation resources
* `/code`: Location of the implementation
* `/missions`: Missions used for evaluation of the masters thesis

# Getting started
Each agent needs to set up a ROS 2 environment. BDL is tested with ROS 2 Dashing and Crystal.
## Serial Watchdog
Each agent needs to run an instance of the `serial-watchdog`-script. It automatically starts a DDS-Agent for each connected serial device. This enables new sensors and devices to be automatically integrated in ROS2 and the BDL-framework.
## Mission Deployment
The mission deployment script automatically pulls the git repo containing the mission files and executes its mission after retrieving the latest mission.
Note that the paths are hardcoded and might need adjustment on your device.
You can automatically start the script on boot by adding the following to your crontab:
`@reboot screen -dm /path/to/start_mission.sh`

## Mission Player
The Mission Player executes the mission files provided by the user. It's functionality can be extended with plugins. By default a plugin for interfacing with ROS and a simple synchronization-plugin are provided.
## Swarm Synchronization
The `swarm_sync`-package provides a simple way to synchronize multiple agents running BDL and ROS2. It is implemented as a ROS2 package that uses the DDS communication of ROS2 to create simple barriers where agents can pause their missions until all other agents have reached the same barrier.

# Creating Sensors
For integrating new sensors into BDL, the `SmartSensorLib` was created. It is located in the `SmartSensors` folder of the code directory alongside with sample implementations for the `DHT22`, `DS18B20` and `PT1000`-sensors.
