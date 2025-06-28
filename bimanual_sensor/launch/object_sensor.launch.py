from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    camera_placeholder = LaunchConfiguration('camera_placeholder')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('camera_placeholder', default_value='true'),

        Node(
            package='bimanual_sensor',
            executable='object_sensor_node',
            name='object_sensor',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('bimanual_sensor'), 'config/cv_params.yaml')]
        ),

        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('bimanual_sensor'), 'config', 'object_sensor.rviz')],
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('bimanual_mockup'), 'launch/world_mockup.launch.py')
            ]),
            launch_arguments={
                'use_rviz': 'false',  # suppress RViz from world_mockup
                'camera_placeholder': camera_placeholder,
            }.items()
        ),
    ])
