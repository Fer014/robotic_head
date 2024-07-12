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

#ifndef READ_WRITE_NODE_HPP_
#define READ_WRITE_NODE_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "sensor_msgs/msg/joint_state.hpp"

class ReadWriteNode : public rclcpp::Node
{
public:

  ReadWriteNode();
  virtual ~ReadWriteNode();

private:
  void timer_callback();
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr set_position_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr tf_angles_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

};

#endif  // READ_WRITE_NODE_HPP_
