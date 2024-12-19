
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import yaml

def generate_launch_description():
    ld = LaunchDescription()

    emergency_braking_node = Node(
        package='emergency_braking',
        executable='emergency_braking',
        output='screen',
        name='emergency_braking',
        parameters=[]
    )


    # finalize
    ld.add_action(emergency_braking_node)

    return ld
