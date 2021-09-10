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
#ifndef REFERENCE_SYSTEM__NODE__REACTOR_HPP_
#define REFERENCE_SYSTEM__NODE__REACTOR_HPP_
#pragma once

#include <chrono>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "reference_system/node_settings.hpp"
#include "reference_system/number_cruncher.hpp"
#include "reference_system/sample_management.hpp"
#include "reference_system/types.hpp"

namespace node
{

class Reactor : public rclcpp::Node
{
public:
  explicit Reactor(const ReactorSettings & settings)
  : Node(settings.node_name),
    number_crunch_time_(settings.number_crunch_time)
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
    (void)input_number;
    auto number_cruncher_result = number_cruncher(number_crunch_time_);

    auto output_message = publisher_->borrow_loaned_message();

    fuse_samples(this->get_name(), output_message.get(), input_message);

    output_message.get().data[0] = number_cruncher_result.empty();
    publisher_->publish(std::move(output_message));
  }

private:
  publisher_t publisher_;
  std::vector<subscription_t> subscriptions_;
  std::chrono::nanoseconds number_crunch_time_;
};
}  // namespace node

#endif  // REFERENCE_SYSTEM__NODE__REACTOR_HPP_
