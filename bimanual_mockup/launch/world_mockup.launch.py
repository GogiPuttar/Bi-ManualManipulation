from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the config folder
    pkg_share = get_package_share_directory('bimanual_mockup')
    config_dir = os.path.join(pkg_share, 'config')

    use_rviz = LaunchConfiguration('use_rviz')
    camera_placeholder = LaunchConfiguration('camera_placeholder')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('camera_placeholder', default_value='true'),
        DeclareLaunchArgument('camera_params_file', default_value='config/camera_params.yaml'),
        DeclareLaunchArgument('object_params_file', default_value='config/object_params.yaml'),
        DeclareLaunchArgument('world_params_file', default_value='config/world_params.yaml'),

        Node(
            package='bimanual_mockup',
            executable='world_mockup',
            name='world_mockup',
            output='screen',
            parameters=[{
                'camera_params_file': os.path.join(config_dir, 'camera_params.yaml'),
                'object_params_file': os.path.join(config_dir, 'object_params.yaml'),
                'world_params_file': os.path.join(config_dir, 'world_params.yaml'),
                'use_rviz': use_rviz,
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(use_rviz),
            arguments=['-d', os.path.join(config_dir, 'world_mockup.rviz')]
        ),

        # ---- Left Camera Placeholder TF ----
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='left_cam_tf',
            condition=IfCondition(camera_placeholder),
            arguments=['-0.6', '0.6', '1.0', '1', '0', '0', '0', 'world', 'left/camera']
        ),

        # ---- Right Camera Placeholder TF ----
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='right_cam_tf',
            condition=IfCondition(camera_placeholder),
            arguments=['0.6', '0.6', '1.0', '1', '0', '0', '0', 'world', 'right/camera']
        ),
    ])
