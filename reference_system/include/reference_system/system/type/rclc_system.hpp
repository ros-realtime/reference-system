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
#ifndef REFERENCE_SYSTEM__SYSTEM__TYPE__RCLC_SYSTEM_HPP_
#define REFERENCE_SYSTEM__SYSTEM__TYPE__RCLC_SYSTEM_HPP_
#include "reference_system/nodes/rclc/command.hpp"
#include "reference_system/nodes/rclc/fusion.hpp"
#include "reference_system/nodes/rclc/transform.hpp"
#include "reference_system/nodes/rclc/cyclic.hpp"
#include "reference_system/nodes/rclc/sensor.hpp"
#include "reference_system/nodes/rclc/intersection.hpp"

struct RclcSystem
{
  // using NodeBaseType = nodes::rclc_system::NodeBase;

  using Command = nodes::rclc_system::Command;
  using Cyclic = nodes::rclc_system::Cyclic;
  using Fusion = nodes::rclc_system::Fusion;
  using Intersection = nodes::rclc_system::Intersection;
  using Sensor = nodes::rclc_system::Sensor;
  using Transform = nodes::rclc_system::Transform;
};

#endif  // REFERENCE_SYSTEM__SYSTEM__TYPE__RCLC_SYSTEM_HPP_
