from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_takeoff_pkg',
            executable='takeoff_node',
            output='screen'
        )
    ])
