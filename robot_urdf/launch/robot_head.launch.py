# main_launch.py

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Include the URDF and RViz launch file
    urdf_rviz_launch_file = PathJoinSubstitution([
        FindPackageShare('robot_urdf'),  # Replace with your package name if different
        'launch',
        'display.launch.py'
    ])
    
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(urdf_rviz_launch_file),
            launch_arguments={
                'gui': 'false',
                'model': 'urdf/head.urdf',
                'package': 'robot_urdf',
                'rvizconfig': os.path.join(
                    os.getcwd(), 'src', 'robotic_head', 'robot_urdf', 'config', 'head.rviz'
                )
            }.items()
        )
    )

    # Add the read_write_node to the launch description
    ld.add_action(
        Node(
            package='dynamixel_sdk_examples',
            executable='read_write_node',
            output='screen',
            name='read_write_node'
        )
    )

    # Add the static_tf_publisher node to the launch description
    ld.add_action(
        Node(
            package='dynamixel_sdk_examples',
            executable='static_tf_publisher',
            output='screen',
            name='static_tf_publisher'
        )
    )

    # Path to the realsense2_camera launch file
    realsense_launch_file = os.path.join(
        os.getcwd(), 'install', 'realsense2_camera', 'share', 'realsense2_camera', 'launch', 'rs_launch.py'
    )

    # Include the realsense2_camera launch file (D435_1)
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_file),
            launch_arguments={
                'camera_namespace': 'robot1',
                'camera_name': 'D435_head',
                'serial_no': '_241122071360',
                'depth_module.profile': '1280x720x30',
                'pointcloud.enable': 'true'
            }.items()
        )
    )

    # Include the realsense2_camera launch file (D435_2)
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_file),
            launch_arguments={
                'camera_namespace': 'robot1',
                'camera_name': 'D435_ceiling',
                'serial_no': '_239722072963',
                'depth_module.profile': '1280x720x30',
                'pointcloud.enable': 'true'
            }.items()
        )
    )

    return ld
