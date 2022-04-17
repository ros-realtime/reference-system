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
#ifndef REFERENCE_SYSTEM__NODES__RCLCPP__INTERSECTION_HPP_
#define REFERENCE_SYSTEM__NODES__RCLCPP__INTERSECTION_HPP_
#include <chrono>
#include <string>
#include <utility>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "reference_system/msg_types.hpp"
#include "reference_system/nodes/settings.hpp"
#include "reference_system/number_cruncher.hpp"
#include "reference_system/sample_management.hpp"

namespace nodes
{
namespace rclcpp_system
{

class Intersection : public rclcpp::Node
{
public:
  explicit Intersection(const IntersectionSettings & settings)
  : Node(settings.node_name)
  {
    for (auto & connection : settings.connections) {
      rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> options;
      rclcpp::CallbackGroup::SharedPtr callback_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
      options.callback_group = callback_group;
      connections_.emplace_back(
        Connection{
            this->create_publisher<message_t>(connection.output_topic, 1),
            this->create_subscription<message_t>(
              connection.input_topic, 1,
              [this, id = connections_.size()](const message_t::SharedPtr msg) {
                input_callback(msg, id);
              }),
            callback_group,
            connection.number_crunch_limit});
    }
  }
  rclcpp::CallbackGroup::SharedPtr get_callback_group_of_subscription(
    const std::string & input_topic)
  {
    for (auto & connection : connections_) {
      if (input_topic == connection.subscription->get_topic_name()) {
        return connection.callback_group;
      }
    }
    RCLCPP_FATAL(get_logger(), "Subscription for topic '%s' not found!", input_topic.c_str());
    std::exit(1);
  }

private:
  void input_callback(
    const message_t::SharedPtr input_message,
    const uint64_t id)
  {
    uint64_t timestamp = now_as_int();
    auto number_cruncher_result =
      number_cruncher(connections_[id].number_crunch_limit);

    auto output_message = connections_[id].publisher->borrow_loaned_message();
    output_message.get().size = 0;
    merge_history_into_sample(output_message.get(), input_message);

    uint32_t missed_samples = get_missed_samples_and_update_seq_nr(
      input_message, connections_[id].input_sequence_number);

    set_sample(
      this->get_name(), connections_[id].sequence_number++,
      missed_samples, timestamp, output_message.get());

    // use result so that it is not optimizied away by some clever compiler
    output_message.get().data[0] = number_cruncher_result;
    connections_[id].publisher->publish(std::move(output_message));
  }

private:
  struct Connection
  {
    rclcpp::Publisher<message_t>::SharedPtr publisher;
    rclcpp::Subscription<message_t>::SharedPtr subscription;
    rclcpp::CallbackGroup::SharedPtr callback_group;
    uint64_t number_crunch_limit;
    uint32_t sequence_number = 0;
    uint32_t input_sequence_number = 0;
  };
  std::vector<Connection> connections_;
};
}  // namespace rclcpp_system
}  // namespace nodes
#endif  // REFERENCE_SYSTEM__NODES__RCLCPP__INTERSECTION_HPP_
