cmake_minimum_required(VERSION 3.8)
project(robot_isaac_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(test_ros2bridge src/test_ros2bridge.cpp)
ament_target_dependencies(test_ros2bridge rclcpp sensor_msgs)

add_executable(digital_twin_bridge src/digital_twin_bridge.cpp)
ament_target_dependencies(digital_twin_bridge 
  rclcpp 
  sensor_msgs
)

install(DIRECTORY
  src
  scripts
  launch
  DESTINATION share/${PROJECT_NAME})

# Install C++ executables
install(TARGETS
  test_ros2bridge
  digital_twin_bridge
  DESTINATION lib/${PROJECT_NAME})
  
# Install Python executables
install(PROGRAMS
  scripts/run_isaacsim.py
  scripts/open_isaacsim_stage.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

