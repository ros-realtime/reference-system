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
#include "reference_system/msg_types.hpp"
#include "reference_system/nodes/settings.hpp"
#include "reference_system/number_cruncher.hpp"
#include "reference_system/sample_management.hpp"

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
        subscription_t{
            this->create_subscription<message_t>(
              input_topic, 1,
              [this, input_number](const message_t::SharedPtr msg) {
                input_callback(input_number, msg);
              }),
            0, message_t::SharedPtr()});
      ++input_number;
    }
    publisher_ = this->create_publisher<message_t>(settings.output_topic, 1);
    timer_ = this->create_wall_timer(
      settings.cycle_time,
      [this] {timer_callback();});
  }

private:
  void input_callback(
    const uint64_t input_number,
    const message_t::SharedPtr input_message)
  {
    subscriptions_[input_number].cache = input_message;
  }

  void timer_callback()
  {
    uint64_t timestamp = now_as_int();
    auto number_cruncher_result = number_cruncher(number_crunch_limit_);

    auto output_message = publisher_->borrow_loaned_message();
    output_message.get().size = 0;

    uint32_t missed_samples = 0;
    for (auto & s : subscriptions_) {
      if (!s.cache) {
        continue;
      }

      missed_samples +=
        get_missed_samples_and_update_seq_nr(s.cache, s.sequence_number);

      merge_history_into_sample(output_message.get(), s.cache);
      s.cache.reset();
    }
    set_sample(
      this->get_name(), sequence_number_++, missed_samples, timestamp,
      output_message.get());

    output_message.get().data[0] = number_cruncher_result;
    publisher_->publish(std::move(output_message));
  }

private:
  rclcpp::Publisher<message_t>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  struct subscription_t
  {
    rclcpp::Subscription<message_t>::SharedPtr subscription;
    uint32_t sequence_number = 0;
    message_t::SharedPtr cache;
  };

  std::vector<subscription_t> subscriptions_;
  uint64_t number_crunch_limit_;
  uint32_t sequence_number_ = 0;
};
}  // namespace rclcpp_system
}  // namespace nodes
#endif  // REFERENCE_SYSTEM__NODES__RCLCPP__CYCLIC_HPP_
