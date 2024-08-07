// Copyright 2021 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*******************************************************************************
// This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
// For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
// To test this example, please follow the commands below.
//
// Open terminal #1
// $ ros2 run dynamixel_sdk_examples read_write_node
//
// Open terminal #2 (run one of below commands at a time)
// $ ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/SetPosition "{id: 1, position: 1000}"
// $ ros2 service call /get_position dynamixel_sdk_custom_interfaces/srv/GetPosition ""
//
// Original author: Will Son
// Modified by: Fernando Moreno
*******************************************************************************/

#include <cstdio>
#include <memory>
#include <string>
#include <chrono>
#include <functional>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "read_write_node.hpp"

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_VELOCITY 128
#define ADDR_PRESENT_POSITION 132
#define ADDR_PROFILE_VELOCITY 112

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define DXL1_ID                         1                   // Dynamixel ID: 1
#define DXL2_ID                         2                   // Dynamixel ID: 2
#define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

// Angle constraints
#define DXL1_ANGLE_LOW  -20
#define DXL1_ANGLE_HIGH  20
#define DXL2_ANGLE_LOW  -50
#define DXL2_ANGLE_HIGH  50
// Default velocity value
#define DEFAULT_VELOCITY 8
#define VELOCITY_RPM_UNIT 0.229


using namespace std::chrono_literals;

int angleToPos(int x, int in_max = 360, int out_max = 4096, int offset = 2048)
{
  return round(x * (float)(out_max) / (in_max) + offset);
}

int PosToAngle(int x, int in_max = 4096, int out_max = 360, int offset = -180)
{
  return round(x * (float)(out_max) / (in_max) + offset);
}

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

uint8_t dxl_error = 0;
uint32_t goal_position = 0;
int dxl_comm_result = COMM_TX_FAIL;


ReadWriteNode::ReadWriteNode()
: Node("read_write_node")
{
  RCLCPP_INFO(this->get_logger(), "Run read write node");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  set_position_subscriber_ =
    this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_command", QOS_RKL10V, [this](const sensor_msgs::msg::JointState::SharedPtr msg) -> void
    {
      uint8_t dxl_error = 0;

      // Angle Constraints
      if(msg->position[0] < DXL1_ANGLE_LOW) msg->position[0] = DXL1_ANGLE_LOW;
      else if(msg->position[0] > DXL1_ANGLE_HIGH) msg->position[0] = DXL1_ANGLE_HIGH;
      if(msg->position[1] < DXL2_ANGLE_LOW) msg->position[1] = DXL2_ANGLE_LOW;
      else if(msg->position[1] > DXL2_ANGLE_HIGH) msg->position[1] = DXL2_ANGLE_HIGH;

      // Position Value of X series is 4 byte data.
      // Position Values are transformed from degrees to Dynamixel Units
      uint32_t goal_position_1 = static_cast<uint32_t>(angleToPos(msg->position[0]));  // Convert int32 -> uint32
      uint32_t goal_position_2 = static_cast<uint32_t>(angleToPos(msg->position[1]));  // Convert int32 -> uint32

      // Check velocity array size and set default values if necessary
      int32_t velocity_1 = (msg->velocity.size() > 0 && msg->velocity[0] != 0) ? static_cast<int32_t>(msg->velocity[0]) : DEFAULT_VELOCITY;
      int32_t velocity_2 = (msg->velocity.size() > 1 && msg->velocity[1] != 0) ? static_cast<int32_t>(msg->velocity[1]) : DEFAULT_VELOCITY;
      // Velocity Values are transformed from RPM (revolutions per minute) to Dynamixel Units
      velocity_1 /= VELOCITY_RPM_UNIT;
      velocity_2 /= VELOCITY_RPM_UNIT;
      packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_PROFILE_VELOCITY, velocity_1);
      packetHandler->write4ByteTxRx(portHandler, DXL2_ID, ADDR_PROFILE_VELOCITY, velocity_2);

      // Write Goal Position 1 (length : 4 bytes)
      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        (uint8_t) DXL1_ID,
        ADDR_GOAL_POSITION,
        goal_position_1,
        &dxl_error
      );

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Angle: %f]", DXL1_ID, msg->position[0]);
      }

      // Write Goal Position 2 (length : 4 bytes)
      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        (uint8_t) DXL2_ID,
        ADDR_GOAL_POSITION,
        goal_position_2,
        &dxl_error
      );

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Angle: %f]", DXL2_ID, msg->position[1]);
      }

    }
    );

  //publisher
  tf_angles_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", QOS_RKL10V);
  timer_ = this->create_wall_timer( 1ms, std::bind(&ReadWriteNode::timer_callback, this));
  

}

ReadWriteNode::~ReadWriteNode()
{
}

void ReadWriteNode::timer_callback()
{
  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.stamp = this->now();
  joint_state_msg.name.resize(2);
  joint_state_msg.position.resize(2);
  joint_state_msg.velocity.resize(2);

  joint_state_msg.name[0] = "neck_dx_joint";
  joint_state_msg.name[1] = "dx_tilt_joint";

    // Read Present Position 1 (length : 4 bytes) and Convert uint32 -> int32
    uint32_t present_position_1, present_position_2, present_velocity_1, present_velocity_2;
    dxl_comm_result = packetHandler->read4ByteTxRx(
      portHandler,
      (uint8_t) DXL1_ID,
      ADDR_PRESENT_POSITION,
      reinterpret_cast<uint32_t *>(&present_position_1),
      &dxl_error
    );
    // Read Present Position 2 (length : 4 bytes) and Convert uint32 -> int32
    dxl_comm_result = packetHandler->read4ByteTxRx(
      portHandler,
      (uint8_t) DXL2_ID,
      ADDR_PRESENT_POSITION,
      reinterpret_cast<uint32_t *>(&present_position_2),
      &dxl_error
    );

    // Read Present Velocity 1 (length : 4 bytes) and Convert uint32 -> int32
    dxl_comm_result = packetHandler->read4ByteTxRx(
      portHandler, (
      uint8_t) DXL1_ID, 
      ADDR_PRESENT_VELOCITY, 
      reinterpret_cast<uint32_t *>(&present_velocity_1),
      &dxl_error
    );

    // Read Present Velocity 2 (length : 4 bytes) and Convert uint32 -> int32
    dxl_comm_result = packetHandler->read4ByteTxRx(
      portHandler, (
      uint8_t) DXL2_ID, 
      ADDR_PRESENT_VELOCITY, 
      reinterpret_cast<uint32_t *>(&present_velocity_2),
      &dxl_error
    );

    // Position Values are transformed from Dynamixel Units to degrees
    joint_state_msg.position[0] = PosToAngle(present_position_1);
    joint_state_msg.position[1] = PosToAngle(present_position_2);

    // Velocity Values are transformed from Dynamixel Units to RPM (revolutions per minute)
    joint_state_msg.velocity[0] = present_velocity_1 * VELOCITY_RPM_UNIT;
    joint_state_msg.velocity[1] = present_velocity_2 * VELOCITY_RPM_UNIT;

    tf_angles_publisher_->publish(joint_state_msg);
}


void setupDynamixel(uint8_t dxl_id)
{
  // Use Position Control Mode
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_OPERATING_MODE,
    3,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set Position Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set Position Control Mode.");
  }

  // Change Dynamixel Profile Velocity
  packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_PROFILE_VELOCITY, 15);

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to enable torque.");
  }
}

int main(int argc, char * argv[])
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set the baudrate.");
  }

  setupDynamixel(BROADCAST_ID);

  rclcpp::init(argc, argv);

  auto readwritenode = std::make_shared<ReadWriteNode>();
  rclcpp::spin(readwritenode);
  rclcpp::shutdown();

  // Disable Torque of DYNAMIXEL
  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );

  return 0;
}