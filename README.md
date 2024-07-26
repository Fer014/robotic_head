# robotic_head

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)


## Installation
To get a local copy up and running, follow these simple steps:

### Prerequisites

- [ROS2 Humble] - https://docs.ros.org/en/humble/Installation.html

It may be necessary to install Xacro:
```bash
sudo apt-get update
sudo apt install ros-humble-xacro
```
- [Intel RealSense ROS2 Wrapper] - https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#installation-on-ubuntu
  
Make sure to follow the tutorial: in step 2 choose option 1, in step 3 choose option 2
- [DYNAMIXEL Wizard 2.0] (optional) - https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/

To enable read and write permissions for the port where the Dynamixel is connected:
```bash
sudo chmod a+rw /dev/ttyUSB0
```

### Installation steps
1. **Create a ROS2 workspace (or use an existing one):**
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src/
```
2. **Clone the repository:**
```bash
git clone https://github.com/Fer014/robotic_head.git
cd ~/ros2_ws
```
3. **Build and source:**
```bash
colcon build
source install/setup.bash 
```

## Usage
### Launch files
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
### Run programs
Run program to perform a scanning test moving the robot head
```bash
ros2 run robot_isaac_sim scan_joint_state_publisher
```
### Topics
Set motors position in both real and simulated robot (velocity values are optional)
