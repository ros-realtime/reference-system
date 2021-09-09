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

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "reference_system_autoware/sample_management.hpp"
#include "reference_system_autoware/types.hpp"

namespace node {
struct CommandSettings {
  std::string node_name;
  std::string input_topic;
};

class Command : public rclcpp::Node {
 public:
  Command(const CommandSettings& settings) : Node(settings.node_name) {
    subscription_ = this->create_subscription<message_t>(
        settings.input_topic, 10,
        [this](const message_t::SharedPtr msg) { input_callback(msg); });
  }

 private:
  void input_callback(const message_t::SharedPtr input_message) const {
    print_sample_path(this->get_name(), input_message);
  }

 private:
  subscription_t subscription_;
};
}  // namespace node
