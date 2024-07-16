# robotic_head

## Steps to install
1) Clone the repo  
2) Colcon build
3) Source the repo

## Launch files
Launch real robot urdf in RVIZ
```bash
ros2 launch robot_urdf display.launch.py
```
Launch simulated robot in Isaac Sim
```bash
ros2 launch robot_isaac_sim run_isaacsim.launch.py
```
Launch object recognition and head tracking program
```bash
ros2 launch dnn_yolo dnn_yolo.launch.py
```
## Run programs
Run program to perform a scanning test moving the robot head
```bash
ros2 run robot_isaac_sim scan_joint_state_publisher
```
## Topics
Set motors position in both real and simulated robot (velocity values are optional)
```bash
ros2 topic pub -1 /joint_command sensor_msgs/JointState '{name: ["neck_dx_joint", "dx_tilt_joint"], position: [0, 0]}'
```
