// Copyright 2021 Apex.AI, Inc.
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
#ifndef REFERENCE_SYSTEM__SYSTEM__TYPE__AUTOWARE__RCLCPP_SYSTEM_HPP_
#define REFERENCE_SYSTEM__SYSTEM__TYPE__AUTOWARE__RCLCPP_SYSTEM_HPP_
#include "reference_system/nodes/rclcpp/command.hpp"
#include "reference_system/nodes/rclcpp/fusion.hpp"
#include "reference_system/nodes/rclcpp/processing.hpp"
#include "reference_system/nodes/rclcpp/reactor.hpp"
#include "reference_system/nodes/rclcpp/sensor.hpp"

struct AutowareRclcppSystem
{
  using NodeBaseType = rclcpp::Node;
  using Sensor = nodes::rclcpp_system::Sensor;
  using Command = nodes::rclcpp_system::Command;
  using Fusion = nodes::rclcpp_system::Fusion;
  using Reactor = nodes::rclcpp_system::Reactor;
  using Processing = nodes::rclcpp_system::Processing;
};

#endif  // REFERENCE_SYSTEM__SYSTEM__TYPE__AUTOWARE__RCLCPP_SYSTEM_HPP_
