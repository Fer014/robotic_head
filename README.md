# robotic_head

## Steps to install
1) Clone repo  
2) Colcon build

## Launch files
Launch robot urdf in RVIZ
```bash
ros2 launch robot_urdf display.launch.py
```
Launch object recognition and head tracking
```bash
ros2 launch dnn_yolo dnn_yolo.launch.py
```
## Topics
Set motors position (velocity values are optional)
```bash
ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/SetPosition "{angle_1: 0, angle_2: 0, velocity_1: 30, velocity_2: 30}"
```
