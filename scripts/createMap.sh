#!/bin/bash

# Default arguments
build_flag=false
hold_flag=false
open_flag=false
sim_flag=false
loc_flag=false
loc_config="mapping_localization/mapper_params_online_async.yaml"   # default vehicle mapping config
path="/home/itse/atp_f1tenth_racecar_ws"

# Default sim mapping config
sim_cfg="mapping_localization/mapper_params_simulation_async.yaml"

# Get arguments
while getopts 'obhsp:l:' flag; do
  case "${flag}" in
    b) build_flag=true ;;
    h) hold_flag=true ;;
    o) open_flag=true ;;
    s) sim_flag=true ;;
    l) loc_config="${OPTARG}" ; loc_flag=true ;;
    p) path="${OPTARG}" ;;
  esac
done

# Name session
SESSION=createMap
if [[ $open_flag == false ]]; then
    echo "TO ATTACH TO SESSION: screen -r ${SESSION}";
fi

# Hold window after execution
HOLD=''
if [[ $hold_flag == true ]]; then
    HOLD="exec bash" ;
fi

# Navigate to ROS2 workspace
cd $path

# Build workspace if required
if [[ $build_flag == true ]]; then
    colcon build;
fi

# Source workspace
source /opt/ros/iron/setup.bash
source install/setup.bash

# Create detached session for launching vehicle control
CMD1="ros2 launch vehicle_control vehicle_control_launch.py"
if [[ $sim_flag == true ]]; then
    CMD1="ros2 launch vehicle_control vehicle_control_simulation_launch.py" ;
fi
screen -dmS ${SESSION} bash -c "${CMD1} ; ${HOLD}"

# Start rviz2 in a new window, by sending a remote command
if [[ $sim_flag == false ]]; then
    screen -S $SESSION -X screen bash -c "ros2 run rviz2 rviz2 ; ${HOLD}";
fi

# Start slam toolbox in a new window, by sending a remote command
LINK=$loc_config
if [[ $sim_flag == true ]] && [[ $loc_flag == false ]]; then
    LINK=$sim_cfg
fi
CMD2="ros2 launch slam_toolbox online_async_launch.py slam_params_file:=${LINK}"
screen -S $SESSION -X screen bash -c "${CMD2} ; ${HOLD}"

# Connect to detached screen session
if [[ $open_flag == true ]]; then 
    screen -r $SESSION;
fi