#!/bin/bash
eval echo "[greenball] starting ... " $toStartlog
source /home/unitree/Unitree/autostart/greenball/devel/setup.sh
roslaunch greenball_tracking greenball.launch
