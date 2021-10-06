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
#ifndef REFERENCE_SYSTEM__NODES__RCLCPP__CYCLIC_HPP_
#define REFERENCE_SYSTEM__NODES__RCLCPP__CYCLIC_HPP_
#include <chrono>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "reference_system/nodes/settings.hpp"
#include "reference_system/number_cruncher.hpp"
#include "reference_system/sample_management.hpp"
#include "reference_system/msg_types.hpp"

namespace nodes
{
namespace rclcpp_system
{

class Cyclic : public rclcpp::Node
{
public:
  explicit Cyclic(const CyclicSettings & settings)
  : Node(settings.node_name),
    number_crunch_limit_(settings.number_crunch_limit)
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
    message_cache_.resize(subscriptions_.size());
    publisher_ = this->create_publisher<message_t>(settings.output_topic, 10);
    timer_ = this->create_wall_timer(
      settings.cycle_time,
      [this] {timer_callback();});
  }

private:
  void input_callback(
    const uint64_t input_number,
    const message_t::SharedPtr input_message)
  {
    message_cache_[input_number] = input_message;
  }

  void timer_callback()
  {
    auto local_cache = message_cache_;
    for (auto & m : message_cache_) {
      m.reset();
    }

    auto number_cruncher_result = number_cruncher(number_crunch_limit_);

    uint64_t sent_samples = 0;
    for (auto & m : local_cache) {
      if (!m) {continue;}

      auto output_message = publisher_->borrow_loaned_message();

      fuse_samples(this->get_name(), output_message.get(), m);
      output_message.get().data[0] = number_cruncher_result;

      publisher_->publish(std::move(output_message));
      m.reset();
      ++sent_samples;
    }

    if (sent_samples == 0) {
      auto message = publisher_->borrow_loaned_message();
      message.get().size = 0;

      set_sample(this->get_name(), message.get());
      message.get().data[0] = number_cruncher_result;

      publisher_->publish(std::move(message));
    }
  }

private:
  rclcpp::Publisher<message_t>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<rclcpp::Subscription<message_t>::SharedPtr> subscriptions_;
  std::vector<message_t::SharedPtr> message_cache_;
  uint64_t number_crunch_limit_;
};
}  // namespace rclcpp_system
}  // namespace nodes
#endif  // REFERENCE_SYSTEM__NODES__RCLCPP__CYCLIC_HPP_
