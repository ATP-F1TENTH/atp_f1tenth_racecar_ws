#!/bin/bash

# Need to positively identify the session name:
SESSION=createMap
echo "TO ATTACH TO SESSION: screen -r ${SESSION}"

CMD1="ros2 launch vehicle_control vehicle_control_launch.py ; exec bash";
CAR=true
if [[ $3 == '--simulation' ]] || [[ $3 == '-S' ]]; then 
    CMD1="ros2 launch vehicle_control vehicle_control_simulation_launch.py"
    CAR=false;
else
    echo "No valid parameter: ${3}. Starting real car launch...";
fi

# Setup ROS2 environment
cd /home/itse/atp_f1tenth_racecar_ws
if [[ $2 == '--build' ]] || [[ $2 == '-B' ]]; then
    colcon build;
else
    echo "No valid parameter: ${2}. Build process deactivated.";
fi
source /opt/ros/iron/setup.bash
source install/setup.bash

# Create detached session for rviz2
screen \
    -d -m \
    -S ${SESSION} \
    bash -c "${CMD1}"

# Now start vehicle control in a new window, by sending a remote command:
if [[ $CAR == true ]]; then
    screen -S $SESSION -X screen bash -c "ros2 run rviz2 rviz2";
fi

# Now start slam toolbox in a new window, by sending a remote command:
CMD2="ros2 launch slam_toolbox online_async_launch.py slam_params_file:=mapping_localization/mapper_params_simulation_async.yaml"
screen -S $SESSION -X screen bash -c "${CMD2}"

if [[ $1 == '--open' ]] || [[ $1 == '-O' ]]; then 
    screen -r $SESSION;
else
    echo "No valid parameter: ${1}. Session detached.";
fi