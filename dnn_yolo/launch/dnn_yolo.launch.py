from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Define the node from dnn_yolo
    dnn_yolo_node = Node(
        package='dnn_yolo',
        executable='DNN_YOLO',
        name='dnn_yolo',
        output='screen'
    )
    
    # Define the first node from dynamixel_sdk_examples
    read_write_node = Node(
        package='dynamixel_sdk_examples',
        executable='read_write_node',
        name='read_write_node',
        output='screen'
    )

    # Define the second node from dynamixel_sdk_examples
    set_position_publisher = Node(
        package='dynamixel_sdk_examples',
        executable='set_position_publisher',
        name='set_position_publisher',
        output='screen'
    )


    # Create the launch description and populate it with the nodes
    ld = LaunchDescription()
    ld.add_action(read_write_node)
    ld.add_action(set_position_publisher)
    ld.add_action(dnn_yolo_node)

    return ld

if __name__ == '__main__':
    generate_launch_description()
