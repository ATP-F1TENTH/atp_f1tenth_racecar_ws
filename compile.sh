#!/bin/bash

# Default arguments
packages_flag=false
screen_flag=false
launch_file_flag=false
hold_flag=false

launch_file="vehicle_control_simulation_launch.py"

# Get arguments
while getopts 'hpsl:f' flag; do
  case "${flag}" in
    h) hold_flag=true ;;
    p) packages_flag=true ;;
    s) screen_flag=true ;;
    l) launch_file="${OPTARG}" ; launch_file_flag=true ;;
  esac
done

# Hold window after execution
HOLD=''
if $hold_flag == true
then
    HOLD="exec bash" ;
fi

if $packages_flag == true
then
    colcon build --packages-select vehicle_control mpcc raceline;   # gap_follower pure_pursuit;
else
    colcon build;
fi

source /opt/ros/iron/setup.bash
source install/setup.bash

if $screen_flag == true
then
    # Name session
    SESSION=createMap;
    CMD1="ros2 launch vehicle_control ${launch_file}";
    screen -dmS ${SESSION} bash -c "${CMD1} ; ${HOLD}";
    launch_node="mapping_localization/localization_launch_amcl.py";
    file="mapping_localization/nav2_params.yaml";
    map="/home/itse/atp_f1tenth_racecar_ws/src/f1tenth_gym_ros/maps/levine_blocked.yaml";
    CMD2="ros2 launch ${launch_node} params_file:=${file} map:=${map}";
    screen -S $SESSION -X screen bash -c "${CMD2} ; ${HOLD}";
    screen -r $SESSION;
else
    ros2 launch vehicle_control ${launch_file}
fi