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
#include "reference_system_autoware/number_cruncher.hpp"
#include "reference_system_autoware/types.hpp"

namespace node
{
struct ReactorSettings
{
  std::string node_name;
  std::vector<std::string> inputs;
  std::string output_topic;
};

class Reactor : public rclcpp::Node
{
public:
  Reactor(const ReactorSettings & settings)
  : Node(settings.node_name)
  {
    uint64_t input_number = 0U;
    for (const auto & input_topic : settings.inputs) {
      subscriptions_.emplace_back(
        this->create_subscription<message_t>(
          input_topic, 10,
          [this, input_number](const message_t::SharedPtr msg) {
            input_callback(input_number, msg);
          }));
      ++input_number;
    }
    publisher_ = this->create_publisher<message_t>(settings.output_topic, 10);
  }

private:
  void input_callback(
    const uint64_t input_number,
    const message_t::SharedPtr input_message) const
  {
    auto output_message = publisher_->borrow_loaned_message();
    int64_t timestamp_in_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch())
      .count();

    const int64_t input_accumulated_latency = input_message->data[0];
    const int64_t input_timestamp = input_message->data[1];

    const int64_t accumulated_latency_in_ns =
      input_accumulated_latency + timestamp_in_ns - input_timestamp;

    output_message.get().data[0] = accumulated_latency_in_ns;
    output_message.get().data[1] = timestamp_in_ns;
    output_message.get().data[2] = input_number;
    publisher_->publish(std::move(output_message));
  }

private:
  publisher_t publisher_;
  std::vector<subscription_t> subscriptions_;
};
}  // namespace node
