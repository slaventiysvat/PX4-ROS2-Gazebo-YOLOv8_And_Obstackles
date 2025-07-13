# ws_offboard_control/src/sptools/launch/minimal_takeoff.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sptools',
            executable='minimal_takeoff',
            output='screen'
        )
    ])
