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

    left_offset = ['-0.3', '0.0', '0.0', '0.0', '0.0', '0.7071', '0.7071'] # [x, y, z, qx, qy, qz, qw]
    right_offset = ['0.3', '0.0', '0.0', '0.0', '0.0', '0.7071', '0.7071'] # [x, y, z, qx, qy, qz, qw]

    # Paths to left and right limb xacros
    left_xacro = PathJoinSubstitution([
        FindPackageShare('bimanual_controller'),
        'urdf', 'left_limb.xacro'
    ])

    right_xacro = PathJoinSubstitution([
        FindPackageShare('bimanual_controller'),
        'urdf', 'right_limb.xacro'
    ])

    # Generate robot descriptions using xacro
    left_robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', left_xacro]),
            value_type=str
        )
    }
    right_robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', right_xacro]),
            value_type=str
        )
    }

    # Left robot state publisher
    left_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='left',
        name='robot_state_publisher',
        parameters=[left_robot_description],
        output='screen'
    )

    # Right robot state publisher
    right_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='right',
        name='robot_state_publisher',
        parameters=[right_robot_description],
        output='screen'
    )

    # OFFSET BROADCASTERS

    # ---- World to Left Offset Link TF ----
    left_offset_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='left_offset_tf',
        # condition=IfCondition(<optional>),
        arguments=[left_offset[0], left_offset[1], left_offset[2], left_offset[3], left_offset[4], left_offset[5], left_offset[6], 'world', 'left_offset_link']
    )

    # ---- World to Right Offset Link TF ----
    right_offset_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='right_offset_tf',
        # condition=IfCondition(<optional>),
        arguments=[right_offset[0], right_offset[1], right_offset[2], right_offset[3], right_offset[4], right_offset[5], right_offset[6], 'world', 'right_offset_link']
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
        left_rsp,
        right_rsp,
        left_offset_tf,
        right_offset_tf,
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
