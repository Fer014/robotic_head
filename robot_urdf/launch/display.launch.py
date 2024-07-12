import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.conditions import IfCondition, UnlessCondition


from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    robot_urdf_path = FindPackageShare('robot_urdf')
    model_path = PathJoinSubstitution(['urdf', 'head.urdf'])
    #rviz_config_path = PathJoinSubstitution([robot_urdf_path, 'config', 'head.rviz'])
    rviz_config_path = os.path.join(os.getcwd(), 'src', 'robot_urdf', 'config', 'head.rviz')

    realsense_launch_file = os.path.join(
        os.getcwd(), 'install', 'realsense2_camera', 'share', 'realsense2_camera', 'launch', 'rs_launch.py'
    )

    # These parameters are maintained for backwards compatibility
    gui_arg = DeclareLaunchArgument(name='gui', default_value='false', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    ld.add_action(gui_arg)

    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=rviz_config_path,
                                     description='Absolute path to rviz config file')
    ld.add_action(rviz_arg)

    # This parameter has changed its meaning slightly from previous versions
    ld.add_action(DeclareLaunchArgument(name='model', default_value=model_path,
                                        description='Path to robot urdf file relative to urdf_tutorial package'))
                                        # This parameter has been added and is not included in the official implementation. Its goal is to provide support for relative path for urdf from cd.
    ld.add_action(DeclareLaunchArgument(name='package', default_value='robot_urdf',
                                        description='Path to robot urdf file relative to cd'))

    package_dir = FindPackageShare(LaunchConfiguration('package'))
    urdf_path = PathJoinSubstitution([package_dir, LaunchConfiguration('model')])

    robot_description_content = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description_content,
                                      }])

    ld.add_action(robot_state_publisher_node)

    # # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    # ld.add_action(Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     condition=UnlessCondition(LaunchConfiguration('gui'))
    # ))

    # ld.add_action(Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     condition=IfCondition(LaunchConfiguration('gui'))
    # ))

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    ))

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

    # Include the realsense2_camera launch file (D435_1)
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_file),
            launch_arguments={
                'camera_namespace': 'robot1',
                'camera_name': 'D435_1',
                'serial_no': '_241122071360',
                'depth_module.profile': '1280x720x30',
                'pointcloud.enable': 'true'
            }.items()
        )
    )

    # # Include the realsense2_camera launch file (D435_2)
    # ld.add_action(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(realsense_launch_file),
    #         launch_arguments={
    #             'camera_namespace': 'robot1',
    #             'camera_name': 'D435_2',
    #             'serial_no': '_239722072963',
    #             'depth_module.profile': '1280x720x30',
    #             'pointcloud.enable': 'true'
    #         }.items()
    #     )
    # )

    return ld
