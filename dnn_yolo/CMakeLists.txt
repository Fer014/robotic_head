cmake_minimum_required(VERSION 3.5)
project(dnn_yolo)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(realsense2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(camera_custom_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)

# find_package(builtin_interfaces REQUIRED)
# find_package(rosidl_default_generators REQUIRED)

# # Generate messages
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Detection.msg"
#   "msg/Detections.msg"
#   DEPENDENCIES builtin_interfaces
# )

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIRS}
  "/usr/include/eigen3/"
)

# Add subdirectory for dlib
add_subdirectory(./dlib dlib_build)

# Executable
add_executable(DNN_YOLO
  src/Main.cpp
  include/Position_Transform.h
  src/Position_Transform.cpp
  src/Realsense_config.cpp
  src/Objection.cpp
  include/Objection.h
  src/Algorithm_Objection_3D.cpp
  include/Algorithm_Objection_3D.h
  src/Visualizer.cpp
  include/Visualizer.h
)

# Link libraries
target_link_libraries(DNN_YOLO
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${realsense2_LIBRARY}
  dlib::dlib
)

ament_target_dependencies(DNN_YOLO
  OpenCV
  Boost
  realsense2
  rclcpp
  sensor_msgs
  ament_index_cpp
  camera_custom_interfaces
)

# # Ensure the executable depends on the generated messages
# add_dependencies(DNN_YOLO
#   ${PROJECT_NAME}_rosidl_generate_interfaces_cpp
# )

# # Include directories for the generated messages
# target_include_directories(DNN_YOLO PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
#   $<INSTALL_INTERFACE:include>
# )

# Install
install(TARGETS 
  DNN_YOLO
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY engine dlib launch
  DESTINATION share/${PROJECT_NAME}
)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ament_export_dependencies(rosidl_default_runtime)
ament_package()
