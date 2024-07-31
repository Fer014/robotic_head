# urdf_rviz_launch.py

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    # Define paths
    robot_urdf_path = FindPackageShare('robot_urdf')
    model_path = PathJoinSubstitution(['urdf', 'head.urdf'])

    # Use absolute path for RViz configuration
    rviz_config_path = os.path.join(
        os.getcwd(), 'src', 'robotic_head', 'robot_urdf', 'config', 'head.rviz'
    )

    # Declare launch arguments
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='false',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    )
    ld.add_action(gui_arg)

    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=rviz_config_path,
        description='Absolute path to rviz config file'
    )
    ld.add_action(rviz_arg)

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=model_path,
        description='Path to robot urdf file relative to urdf_tutorial package'
    )
    ld.add_action(model_arg)

    package_arg = DeclareLaunchArgument(
        name='package',
        default_value='robot_urdf',
        description='Path to robot urdf file relative to cd'
    )
    ld.add_action(package_arg)

    # Resolve paths and launch configurations
    package_dir = FindPackageShare(LaunchConfiguration('package'))
    urdf_path = PathJoinSubstitution([package_dir, LaunchConfiguration('model')])

    # Configure robot description parameter
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_path]), value_type=str
    )

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )
    ld.add_action(robot_state_publisher_node)

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )
    ld.add_action(rviz_node)

    return ld
