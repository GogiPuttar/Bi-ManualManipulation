from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Get the path to the config folder
    pkg_share = get_package_share_directory('bimanual_controller')
    config_dir = os.path.join(pkg_share, 'config')

    use_rviz = LaunchConfiguration('use_rviz')

    # Paths to left and right limb xacros
    system_xacro = PathJoinSubstitution([
        FindPackageShare('bimanual_controller'),
        'urdf', 'bimanual_system.xacro'
    ])

    # Generate robot description using combined xacro
    system_robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', system_xacro]),
            value_type=str
        )
    }

    # System robot state publisher
    system_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[system_robot_description],
        output='screen'
    )

    limbs_mockup_node = Node(
        package='bimanual_controller',
        executable='limbs_mockup_node',
        name='limbs_mockup_node',
        output='screen',
        parameters=[{
            'grasp_params_file': os.path.join(config_dir, 'grasp_params.yaml'),
            'motion_params_file': os.path.join(config_dir, 'motion_params.yaml'),
            **system_robot_description,
            }]
    )

    # RVIZ
    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(config_dir, 'limbs_mockup.rviz')],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to launch RViz'
        ),
        DeclareLaunchArgument(
            'grasp_params_file', 
            default_value='config/grasp_params.yaml'
        ),
        DeclareLaunchArgument(
            'motion_params_file', 
            default_value='config/motion_params.yaml'
        ),
        system_rsp,
        limbs_mockup_node,
        rviz,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('bimanual_sensor'), 'launch/object_sensor.launch.py')
            ]),
            launch_arguments={
                'use_rviz': 'false',  # suppress RViz from world_mockup
                'camera_placeholder': 'false',
            }.items()
        )
    ])
