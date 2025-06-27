from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the config folder
    pkg_share = get_package_share_directory('bimanual_mockup')
    config_dir = os.path.join(pkg_share, 'config')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),

        Node(
            package='bimanual_mockup',
            executable='world_mockup',
            name='world_mockup',
            output='screen',
            parameters=[{
                'camera_params_file': os.path.join(config_dir, 'camera_params.yaml'),
                'object_params_file': os.path.join(config_dir, 'object_params.yaml'),
                'world_params_file': os.path.join(config_dir, 'world_params.yaml'),
                'use_rviz': LaunchConfiguration('use_rviz'),
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            arguments=['-d', os.path.join(config_dir, 'world_mockup.rviz')]
        )
    ])
