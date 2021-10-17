// Copyright 2021.
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
#ifndef REFERENCE_SYSTEM__NODES__RCLC__FUSION_HPP_
#define REFERENCE_SYSTEM__NODES__RCLC__FUSION_HPP_
#include <chrono>
#include <string>
#include <utility>
#include <stdio.h>

#include <rclc/rclc.h>
#include "rclcpp/rclcpp.hpp"

#include "reference_system/nodes/settings.hpp"
#include "reference_system/number_cruncher.hpp"
#include "reference_system/sample_management.hpp"
#include "reference_system/msg_types.hpp"


namespace nodes
{
namespace rclc_system
{

class Fusion
{
public:
  explicit Fusion(const FusionSettings & settings)
  : node_name_(settings.node_name),
    number_crunch_limit_(settings.number_crunch_limit)
  {
    rcl_ret_t rc;

    context_ = rcl_get_zero_initialized_context();
    init_options_ = rcl_get_zero_initialized_init_options();
    allocator_ = rcl_get_default_allocator();

    rc = rcl_init_options_init(&init_options_, allocator_);
    if (rc != RCL_RET_OK) {
      printf("Error rcl_init_options_init.\n");
      return;
    }
    // init rclc - Note: will be done in main with rccpp::init()
    // not sure this will work!
    /*
    rc = rcl_init(argc, argv, &init_options, &context);
    if (rc != RCL_RET_OK) {
      printf("Error in rcl_init.\n");
      return -1;
    }
    */
    
    // create rcl_node
    rcl_node_t node_ = rcl_get_zero_initialized_node();
    rcl_node_options_t node_ops_ = rcl_node_get_default_options();
    rc = rcl_node_init(&node_, settings.node_name.c_str(), "", &context_, &node_ops_);
    if (rc != RCL_RET_OK) {
      printf("Error in rcl_node_init\n");
      return;
    }
    // create subscription A
/*
    //handle local member variables with context variable.
    subscription_[0] = this->create_subscription<message_t>(
      settings.input_0, 10,
      [this](const message_t::SharedPtr msg) {input_callback(0U, msg);});

    // create subscription B
    subscription_[1] = this->create_subscription<message_t>(
      settings.input_1, 10,
      [this](const message_t::SharedPtr msg) {input_callback(1U, msg);});
    
    // create publisher
    publisher_ = this->create_publisher<message_t>(settings.output_topic, 10);
  */
  }

  ~Fusion()
  {
    rcl_ret_t rc;
    rc += rcl_publisher_fini(&publisher_, &node_);
    rc += rcl_subscription_fini(&subscription0_, &node_);
    rc += rcl_subscription_fini(&subscription1_, &node_);
    rc += rcl_node_fini(&node_);
    if (rc != RCL_RET_OK)
    {
      printf("Error calling rcl_*_fini methods\n"); 
    }
  }

private:
  void input_callback(
    const uint64_t input_number,
    const message_t::SharedPtr input_message)
  {
    message_cache_[input_number] = input_message;

    // only process and publish when we can perform an actual fusion, this means
    // we have received a sample from each subscription
    if (!message_cache_[0] || !message_cache_[1]) {
      return;
    }

    auto number_cruncher_result = number_cruncher(number_crunch_limit_);
/*
    auto output_message = publisher_->borrow_loaned_message();
    fuse_samples(
      this->get_name(), output_message.get(), message_cache_[0],
      message_cache_[1]);
    output_message.get().data[0] = number_cruncher_result;
    publisher_->publish(std::move(output_message));
*/
    message_cache_[0].reset();
    message_cache_[1].reset();
  }

private:
  std::string node_name_;

  rcl_node_t node_;
  rcl_context_t context_;
  rcl_init_options_t init_options_;
  rcl_allocator_t allocator_;

  message_t::SharedPtr message_cache_[2];
  rcl_publisher_t publisher_;
  rcl_subscription_t subscription0_;
  rcl_subscription_t subscription1_;

  uint64_t number_crunch_limit_;
};

}  // namespace rclc_system
}  // namespace nodes
#endif  // REFERENCE_SYSTEM__NODES__RCLC__FUSION_HPP_
