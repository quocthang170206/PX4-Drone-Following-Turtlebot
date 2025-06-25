from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yolo_detection_pkg',
            executable='yolo_detection_node',
            output='screen'
        )
    ])

