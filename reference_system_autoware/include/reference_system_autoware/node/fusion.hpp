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
struct FusionSettings
{
  std::string node_name;
  std::string input_0;
  std::string input_1;
  std::string output_topic;
  std::chrono::nanoseconds number_crunch_time;
};

class Fusion : public rclcpp::Node
{
public:
  Fusion(const FusionSettings & settings)
  : Node(settings.node_name),
    number_crunch_time_(settings.number_crunch_time)
  {
    subscription_[0] = this->create_subscription<message_t>(
      settings.input_0, 10,
      [this](const message_t::SharedPtr msg) {input_callback(0U, msg);});

    subscription_[1] = this->create_subscription<message_t>(
      settings.input_1, 10,
      [this](const message_t::SharedPtr msg) {input_callback(1U, msg);});
    publisher_ = this->create_publisher<message_t>(settings.output_topic, 10);
  }

private:
  void input_callback(
    const uint64_t input_number,
    const message_t::SharedPtr input_message)
  {
    input_timestamp_[input_number] = input_message->data[1];
    input_accumulated_latency =
      std::max(input_accumulated_latency, input_message->data[0]);

    // only process and publish when we can perform an actual fusion, this means
    // we have received a sample from each subscription
    if (input_timestamp_[0] == 0U && input_timestamp_[1] == 0U) {return;}

    auto number_cruncher_result = number_cruncher(number_crunch_time_);

    auto output_message = publisher_->borrow_loaned_message();
    int64_t timestamp_in_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch())
      .count();

    int64_t accumulated_latency_in_ns =
      input_accumulated_latency + timestamp_in_ns -
      std::min(input_timestamp_[0], input_timestamp_[1]);

    output_message.get().data[0] = accumulated_latency_in_ns;
    output_message.get().data[1] = timestamp_in_ns;
    // use result so that it is not optimizied away by some clever compiler
    output_message.get().data[2] = number_cruncher_result.empty();
    publisher_->publish(std::move(output_message));

    input_timestamp_[0] = 0U;
    input_timestamp_[1] = 0U;
    input_accumulated_latency = 0U;
  }

private:
  publisher_t publisher_;
  subscription_t subscription_[2];
  int64_t input_accumulated_latency = 0U;
  int64_t input_timestamp_[2]{0U, 0U};

  std::chrono::nanoseconds number_crunch_time_;
};
}  // namespace node
