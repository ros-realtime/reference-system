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

#pragma once

#include "reference_system/reference_nodes/command.hpp"
#include "reference_system/reference_nodes/fusion.hpp"
#include "reference_system/reference_nodes/processing.hpp"
#include "reference_system/reference_nodes/reactor.hpp"
#include "reference_system/reference_nodes/sensor.hpp"

struct ReferenceSystem {
  using NodeBaseType = rclcpp::Node;
  using Sensor = reference_nodes::Sensor;
  using Command = reference_nodes::Command;
  using Fusion = reference_nodes::Fusion;
  using Reactor = reference_nodes::Reactor;
  using Processing = reference_nodes::Processing;
};  // namespace node
