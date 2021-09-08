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
#include "reference_system_autoware/types.hpp"

namespace node
{
struct CommandSettings
{
  std::string node_name;
  std::string input_topic;
};

class Command : public rclcpp::Node
{
public:
  Command(const CommandSettings & settings)
  : Node(settings.node_name)
  {
    subscription_ = this->create_subscription<message_t>(
      settings.input_topic, 10,
      [this](const message_t::SharedPtr msg) {input_callback(msg);});
  }

private:
  void input_callback(const message_t::SharedPtr input_message) const
  {
    const int64_t timestamp_in_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch())
      .count();

    const int64_t input_accumulated_latency = input_message->data[0];
    const int64_t input_timestamp = input_message->data[1];

    const int64_t accumulated_latency_in_ns =
      input_accumulated_latency + timestamp_in_ns - input_timestamp;
    
    RCLCPP_WARN_STREAM(this->get_logger(),
      "Received message statistics:");
    RCLCPP_WARN_STREAM(this->get_logger(),
      "  current timestamp in ns:   " << timestamp_in_ns);
    RCLCPP_WARN_STREAM(this->get_logger(),
      "  message timestamp in ns:   " << input_timestamp);
    RCLCPP_WARN_STREAM(this->get_logger(),
      "  accumulated latency in ns: " << accumulated_latency_in_ns);
  }

private:
  subscription_t subscription_;
};
}  // namespace node
