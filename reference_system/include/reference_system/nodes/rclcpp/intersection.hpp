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

#include "rclcpp/rclcpp.hpp"
#include "reference_system/nodes/settings.hpp"
#include "reference_system/number_cruncher.hpp"
#include "reference_system/sample_management.hpp"
#include "reference_system/msg_types.hpp"

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
      connections_.emplace_back(
        Connection{
            this->create_publisher<message_t>(connection.output_topic, 10),
            this->create_subscription<message_t>(
              connection.input_topic, 10,
              [this, id = connections_.size()](const message_t::SharedPtr msg) {
                input_callback(msg, id);
              }),
            connection.number_crunch_limit
          });
    }
#ifdef PICAS
    connections_[0].subscription->callback_priority = settings.connections[0].callback_priority;
    connections_[1].subscription->callback_priority = settings.connections[1].callback_priority;
#endif
  }

private:
  void input_callback(const message_t::SharedPtr input_message, const uint64_t id) const
  {
    std::cout << "intersection node input msgs id: " << id << std::endl;
    
    auto number_cruncher_result = number_cruncher(connections_[id].number_crunch_limit);

    auto output_message = connections_[id].publisher->borrow_loaned_message();

    fuse_samples(this->get_name(), output_message.get(), input_message);

    // use result so that it is not optimizied away by some clever compiler
    output_message.get().data[0] = number_cruncher_result;
    connections_[id].publisher->publish(std::move(output_message));
  }

private:
  struct Connection
  {
    rclcpp::Publisher<message_t>::SharedPtr publisher;
    rclcpp::Subscription<message_t>::SharedPtr subscription;
    uint64_t number_crunch_limit;
  };
  std::vector<Connection> connections_;
};
}  // namespace rclcpp_system
}  // namespace nodes
#endif  // REFERENCE_SYSTEM__NODES__RCLCPP__INTERSECTION_HPP_
