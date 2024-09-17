# F1tenth workspace

Clone using
```
git clone git@github.com:Autonomous-Racing-Labs/f1tenth_racecar_ws.git
cd f1tenth_racecar_ws
git submodule update --init --remote --merge --recursive
```

Build with colcon:
```
colcon build
```


## Start Simulator
```
source install/setup.bash
ros2 launch vehicle_control vehicle_control_simulation_launch.py
```

## Start on actual vehicle
```
source install/setup.bash
ros2 launch vehicle_control vehicle_control_launch.py
```

# Mapping and Localization

## Extra steps to be done in simulation
When using the simulator, a map is alredy beeing published by the simulator.
As this information clashes with any mapping/localization pipeline, we first need to disable this feature of the simulation.

To this end, open `vehicle_control_simulation_launch.py` from `vehicle_control/launch` and comment out the follwing two lines to stop the simulation's map server from beeing started:
```
#ld.add_action(nav_lifecycle_node)
#ld.add_action(map_server_node)
```


After that, use `colcon build` to build your changes. You may now proceed to Creating a Map using slam_toolbox.


## Create a Map using slam_toolbox
Start Slam Toolbox in the simulation with command
```
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=mapping_localization/mapper_params_simulation_async.yaml
```

Start Slam Toolbox on the actual vehicle with command
```
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=mapping_localization/mapper_params_online_async.yaml
```


Open rviz and add the SlamToobox Panel (Panels --> Add new panels). You can find a "save to file" button there.


### Localize in that map using AMCL Montecarlo Localization
```
ros2 launch mapping_localization/localization_launch_amcl.py params_file:=mapping_localization/nav2_params.yaml map:=MindenCitySpeedway0408.yaml
```


If the map does not show up in rviz2, set Map->Topic->"Durability Policy" to "Transient Local" in the left rviz control pane.

The TF Tree ALWAYS needs to be: map --> odom --> base_link --> laserframe; both for mapping and localization (wheras map --> odom is publiced by the particle filter in localization mode)

## Helpful resources
- https://guni91.wordpress.com/2020/12/05/cartographer-ros2-installation/
- https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html
