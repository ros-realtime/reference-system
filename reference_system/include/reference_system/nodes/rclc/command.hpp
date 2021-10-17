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
#ifndef REFERENCE_SYSTEM__NODES__RCLC__COMMAND_HPP_
#define REFERENCE_SYSTEM__NODES__RCLC__COMMAND_HPP_

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "reference_system/nodes/settings.hpp"
#include "reference_system/sample_management.hpp"
#include "reference_system/msg_types.hpp"

namespace nodes
{
namespace rclc_system
{

class Command : public rclcpp::Node
{
public:
  explicit Command(const CommandSettings & settings)
  : Node(settings.node_name)
  {
    subscription_ = this->create_subscription<message_t>(
      settings.input_topic, 10,
      [this](const message_t::SharedPtr msg) {input_callback(msg);});
  }

private:
  void input_callback(const message_t::SharedPtr input_message)
  {
    uint32_t missed_samples = get_missed_samples_and_update_seq_nr(input_message, sequence_number_);
    print_sample_path(this->get_name(), missed_samples, input_message);
  }

private:
  rclcpp::Subscription<message_t>::SharedPtr subscription_;
  uint32_t sequence_number_ = 0;
};
}  // namespace rclcpp_system
}  // namespace nodes
#endif  // REFERENCE_SYSTEM__NODES__RCLC__COMMAND_HPP_
