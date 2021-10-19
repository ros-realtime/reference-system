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
#ifndef REFERENCE_SYSTEM__SYSTEM__TYPE__RCLC_TRIGGER_SYSTEM_HPP_
#define REFERENCE_SYSTEM__SYSTEM__TYPE__RCLC_TRIGGER_SYSTEM_HPP_
#include "reference_system/nodes/rclc_trigger/command.hpp"
#include "reference_system/nodes/rclc_trigger/fusion.hpp"
#include "reference_system/nodes/rclc_trigger/transform.hpp"
#include "reference_system/nodes/rclc_trigger/cyclic.hpp"
#include "reference_system/nodes/rclc_trigger/sensor.hpp"
#include "reference_system/nodes/rclc_trigger/intersection.hpp"

struct RclcTriggerSystem
{
  using NodeBaseType = rclcpp::Node;

  using Command = nodes::rclc_trigger_system::Command;
  using Cyclic = nodes::rclc_trigger_system::Cyclic;
  using Fusion = nodes::rclc_trigger_system::Fusion;
  using Intersection = nodes::rclc_trigger_system::Intersection;
  using Sensor = nodes::rclc_trigger_system::Sensor;
  using Transform = nodes::rclc_trigger_system::Transform;
};

#endif  // REFERENCE_SYSTEM__SYSTEM__TYPE__RCLC_TRIGGER_SYSTEM_HPP_
