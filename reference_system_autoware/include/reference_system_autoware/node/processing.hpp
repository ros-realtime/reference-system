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
#ifndef REFERENCE_SYSTEM_AUTOWARE__NODE__PROCESSING_HPP_
#define REFERENCE_SYSTEM_AUTOWARE__NODE__PROCESSING_HPP_
#pragma once

#include <chrono>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "reference_system_autoware/number_cruncher.hpp"
#include "reference_system_autoware/sample_management.hpp"
#include "reference_system_autoware/types.hpp"

namespace node
{
struct ProcessingSettings
{
  std::string node_name;
  std::string input_topic;
  std::string output_topic;
  std::chrono::nanoseconds number_crunch_time;
};

class Processing : public rclcpp::Node
{
public:
  explicit Processing(const ProcessingSettings & settings)
  : Node(settings.node_name),
    number_crunch_time_(settings.number_crunch_time)
  {
    subscription_ = this->create_subscription<message_t>(
      settings.input_topic, 10,
      [this](const message_t::SharedPtr msg) {input_callback(msg);});
    publisher_ = this->create_publisher<message_t>(settings.output_topic, 10);
  }

private:
  void input_callback(const message_t::SharedPtr input_message) const
  {
    auto number_cruncher_result = number_cruncher(number_crunch_time_);

    auto output_message = publisher_->borrow_loaned_message();

    fuse_samples(this->get_name(), output_message.get(), input_message);

    // use result so that it is not optimizied away by some clever compiler
    output_message.get().data[2] = number_cruncher_result.empty();
    publisher_->publish(std::move(output_message));
  }

private:
  publisher_t publisher_;
  subscription_t subscription_;
  std::chrono::nanoseconds number_crunch_time_;
};
}  // namespace node

#endif  // REFERENCE_SYSTEM_AUTOWARE__NODE__PROCESSING_HPP_
