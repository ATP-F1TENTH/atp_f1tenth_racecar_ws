# F1tenth workspace

#### Clone using

```
git clone https://github.com/ATP-F1TENTH/atp_f1tenth_racecar_ws.git
```
```
cd atp_f1tenth_racecar_ws
```
```
git submodule update --init --remote --merge --recursive
```
---

#### Build with colcon:
```
colcon build
```


## Start Simulator
```
source install/setup.bash
```
```
ros2 launch vehicle_control vehicle_control_simulation_launch.py
```
When ros2 command is unknown after ***source install/setup.bash*** source underlay first and repeat the prevoius steps afterwards:
```
source /opt/ros/iron/setup.bash
```

## Start on actual vehicle
```
source install/setup.bash
```
```
ros2 launch vehicle_control vehicle_control_launch.py
```

## Mapping and Localization

### Extra steps to be done in simulation
When using the simulator, a map is alredy beeing published by the simulator.
As this information clashes with any mapping / localization pipeline, we first need to disable this feature of the simulation.

To this end, open `vehicle_control_simulation_launch.py` from `vehicle_control/launch` and comment out the follwing two lines to stop the simulation's map server from beeing started:
```
#ld.add_action(nav_lifecycle_node)
#ld.add_action(map_server_node)
```

After that, use `colcon build` to build your changes. You may now proceed to Creating a Map using slam_toolbox.


## Create a Map using slam_toolbox
Start Slam Toolbox in the simulation with command:
```
ros2 launch slam_toolbox online_async_launch.py
```
```
slam_params_file:=mapping_localization/mapper_params_simulation_async.yaml
```

Start Slam Toolbox on the actual vehicle with command:
```
ros2 launch slam_toolbox online_async_launch.py
```
```
slam_params_file:=mapping_localization/mapper_params_online_async.yaml
```

Open rviz and add the SlamToobox Panel (Panels --> Add new panels). You can find a "save to file" button there.


### Automate map creation with slam_toolbox using bash script
Execute bash script `createMap.sh` in direcrtory `/atp_f1tenth_racecar_ws/scripts/` for map creation on actual vehicle:
```
bash ./createMap.sh -o
```
---
The behaviour of this script can be modified using following parameter:
#### Parameter
* `-o` : Flag to connect to detached screen session automatically after execution.
* `-h` : Flag to prevent screen windows from closing after command execution.
* `-b` : Flag to build workspace before start with ***colcon build***.
* `-s` : Flag to use simulation for map creation [no real usecase / for testing purposes].
* `-p` <***ws_path***> : Flag to pass a path to the required workspace beeing used.
* `-s` <***config_path***> : Flag to pass a path to the required config file for the localization.

#### Default values
* <***ws_path***> : The default workspace path is `/home/itse/atp_f1tenth_racecar_ws`
* <***config_path***> :
    1. Start on actual vehicle: `mapping_localization/mapper_params_online_async.yaml`
    2. Start in simulation [used `-s` flag]:  `mapping_localization/mapper_params_simulation_async.yaml`
---

#### Examples
Navigate to the directory containing the script `createMap.sh`
```
cd atp_f1tenth_racecar_ws/scripts/
```
---
Start map creation on actual vehicle, connect to screen session automatically [`-o`], build entire workspace before launching [`-b`] and pass a path to the desired worspace ***/home/itse/new_workspace/***:
```
cd bash ./createMap -o -b -p /home/itse/new_workspace/
```
Or alternative:
```
cd bash ./createMap -ob -p /home/itse/new_workspace/
```
---
Start map creation in simulation [`-s`], connect to screen session automatically [`-o`], prevent screen windows from closing [`-h`] and pass the following path to the config file of the localization: <***/new_workspace/localize/config.yaml***>
```
cd bash ./createMap -s -o -h -l /new_workspace/localize/config.yaml
```
Or alternative:
```
cd bash ./createMap - -soh -l /new_workspace/localize/config.yaml
```
---
#### Usage of screen
* To attach to screen session [`-o` flag not set] use:
    ```screen -r createMap```
* To navigate between session windows use: __Ctr + A__ and then __n__
* To abort executution of current command in window use: __Ctr + c__
* To close window after execution [when flag `-h` is set] use: ```exit```
* For more information visit: https://wiki.ubuntuusers.de/Screen/
___

Stop the localization after the map is ready and add the SlamToobox Panel in rviz (Panels --> Add new panels). You can find a "save to file" button there.

## Localize in that map using AMCL Montecarlo Localization
```
ros2 launch mapping_localization/localization_launch_amcl.py params_file:=mapping_localization/nav2_params.yaml map:=MindenCitySpeedway0408.yaml
```


If the map does not show up in rviz2, set Map->Topic->"Durability Policy" to "Transient Local" in the left rviz control pane.

The TF Tree ALWAYS needs to be: map --> odom --> base_link --> laserframe; both for mapping and localization (wheras map --> odom is publiced by the particle filter in localization mode)

## Helpful resources
- https://guni91.wordpress.com/2020/12/05/cartographer-ros2-installation/
- https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html
