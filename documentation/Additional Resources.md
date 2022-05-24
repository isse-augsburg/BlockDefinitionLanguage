# rosserial
[Arduino Tutorials](http://wiki.ros.org/rosserial_arduino/Tutorials)


# RASPI Config
Rapi 3B, Ubuntu 18.04 64 bit
Remove netplan because it causes problems when cloning images

# Setup Pi
* ros1 melodic und ros2 crystal / disco
* @reboot start_mission.sh in crontab
* mavid.txt in ~




# XRCE install
```bash
sudo apt install libasio-dev
```
git submodule update --init
ln -s thirdparty/fastrtps/include/fastrtps/ include/
ln -s thirdparty/fastcdr/include/fastcdr/ include/
https://micro-xrce-dds.readthedocs.io/en/latest/installation.html



# Serial Watchdog
Usage
```bash
./serial_watchdog.py
```
