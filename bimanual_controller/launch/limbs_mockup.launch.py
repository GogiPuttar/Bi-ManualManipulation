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

def generate_launch_description():

    # Paths to left and right limb xacros
    left_xacro = PathJoinSubstitution([
        FindPackageShare('bimanual_controller'),
        'urdf', 'left_limb.xacro'
    ])

    # right_xacro = PathJoinSubstitution([
    #     FindPackageShare('bimanual_controller'),
    #     'urdf', 'right_limb.xacro'
    # ])

    # Generate robot descriptions using xacro
    left_robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', left_xacro]),
            value_type=str
        )
    }
    # right_robot_description = {'robot_description': Command(['xacro ', right_xacro])}

    # Left robot state publisher
    left_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='left',
        name='robot_state_publisher',
        parameters=[left_robot_description],
        output='screen'
    )

    # # Right robot state publisher
    # right_rsp = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     namespace='right',
    #     name='robot_state_publisher',
    #     parameters=[right_robot_description],
    #     output='screen'
    # )

    return LaunchDescription([
        left_rsp,
        # right_rsp,
    ])
