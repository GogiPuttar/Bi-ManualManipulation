from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="isaac_ros2_bridge",
            executable="ros2_bridge",
            output="screen"
        ),
        Node(
            package="bimanual_isaac",
            executable="spawn_bimanual.py",
            output="screen"
        )
    ])
