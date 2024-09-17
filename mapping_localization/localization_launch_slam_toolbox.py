
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    ld = LaunchDescription()  
    vesc_to_odom_node = Node(
        package="vesc_ackermann",
        executable="vesc_to_odom_node",
        name="vesc_to_odom_node",
        output="screen",
        arguments=['--ros-args'],
        parameters=[{"odom_frame": "odom"}, 
                    {"base_frame": "base_link"}, 
                    {"speed_to_erpm_gain": -4180.0}, 
                    {"speed_to_erpm_offset": 0.0},
                    {"use_servo_cmd_to_calc_angular_velocity": False},
                    {"steering_angle_to_servo_gain": -0.67},
                    {"steering_angle_to_servo_offset": 0.5100},
                    {"wheelbase": 0.33},
                    {"use_imu_to_calc_angular_velocity": True},
                    {"publish_tf": True}]
    )

    localizer = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([
                            FindPackageShare("slam_toolbox"), '/launch', '/online_async_launch.py']),
                        launch_arguments={
                                            "slam_params_file" : "/root/wette_racecar_ws/mapping_localization/localization_params_online_async.yaml"
                                        }.items()
                        )


    # finalize
    #ld.add_action(vesc_to_odom_node) #only required when processing a bag recorded with old vesc_to_odom_node
    ld.add_action(localizer)

    return ld
