#!/bin/bash

pkill -9 -f ros
source install/setup.bash
export ROS_DOMAIN_ID=25
ros2 launch vehicle_control vehicle_control_launch.py


