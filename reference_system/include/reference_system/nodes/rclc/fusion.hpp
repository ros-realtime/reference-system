// Copyright 2021 Robert Bosch GmbH.
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
#include "rclc/rclc.h"
#include "rclc/executor.h"

#include "rclcpp/rclcpp.hpp"
#include "reference_system/nodes/settings.hpp"
#include "reference_system/number_cruncher.hpp"
#include "reference_system/sample_management.hpp"
#include "reference_system/msg_types.hpp"

/*
alternative:
use rosidl types only in the publisher/subscription calls
store as message_t type in this class

this make the code compatible

need to look into rclcpp code, how the conversion between
x::msg::type to x__msg__type is done and apply it here
function is not available in Galactic (only in Rolling)

*/
namespace nodes
{
namespace rclc_system
{

class Fusion : public rclcpp::Node
{
public:
  explicit Fusion(const FusionSettings & settings)
  : Node(settings.node_name),
    number_crunch_limit_(settings.number_crunch_limit)
  {
    rcl_ret_t rc;

    type_support_ = ROSIDL_GET_MSG_TYPE_SUPPORT(reference_interfaces, msg, MESSAGE_T_NAME);
    rc = rclc_subscription_init_default(
      &subscriptions_[0].subscription,
      get_node_base_interface()->get_rcl_node_handle(),
      type_support_,
      settings.input_0.c_str());
    if (rc != RCL_RET_OK) {
      printf("Failed to create subscription %s.\n", settings.input_0.c_str());
      return;
    } else {
      printf("Created subscription %s:\n", settings.input_0.c_str());
    }
    MESSAGE_T_INIT(&msg_0_);

    // TODO(JanStaschulat) set history buffer = 10
    rc = rclc_subscription_init_default(
      &subscriptions_[1].subscription,
      get_node_base_interface()->get_rcl_node_handle(),
      type_support_,
      settings.input_1.c_str());
    if (rc != RCL_RET_OK) {
      printf("Failed to create subscription %s.\n", settings.input_1.c_str());
      return;
    } else {
      printf("Created subscription %s:\n", settings.input_1.c_str());
    }
    MESSAGE_T_INIT(&msg_1_);

    rc = rclc_publisher_init_default(
      &publisher_,
      get_node_base_interface()->get_rcl_node_handle(),
      type_support_,
      settings.output_topic.c_str());
    if (RCL_RET_OK != rc) {
      printf("Error in rclc_publisher_init_default %s.\n", settings.output_topic.c_str());
      return;
    }
    // MESSAGE_T_INIT(&pub_msg_);
  }

  ~Fusion()
  {
    rcl_ret_t rc;
    rc += rcl_publisher_fini(&publisher_, get_node_base_interface()->get_rcl_node_handle());
    rc += rcl_subscription_fini(
      &subscriptions_[0].subscription,
      get_node_base_interface()->get_rcl_node_handle());
    rc += rcl_subscription_fini(
      &subscriptions_[1].subscription,
      get_node_base_interface()->get_rcl_node_handle());

    MESSAGE_T_FINI(&msg_0_);
    MESSAGE_T_FINI(&msg_1_);
    // MESSAGE_T_FINI(&pub_msg_);
    if (rc != RCL_RET_OK) {
      printf("Error calling rcl_*_fini methods\n");
    }
  }

  void add_to_executor(rclc_executor_t * executor)
  {
    rcl_ret_t rc;
    rc = rclc_executor_add_subscription_with_context(
      executor, &subscriptions_[0].subscription, &msg_0_, &input_callback_0, this,
      ON_NEW_DATA);
    if (rc != RCL_RET_OK) {
      printf("Error add_to_executor: subscription0 \n");
    }

    rc = rclc_executor_add_subscription_with_context(
      executor, &subscriptions_[1].subscription, &msg_1_, &input_callback_1, this,
      ON_NEW_DATA);
    if (rc != RCL_RET_OK) {
      printf("Error add_to_executor: subscription1 \n");
    }
    subscriptions_[0].cache = NULL;
    subscriptions_[1].cache = NULL;
  }

private:
  static void input_callback_0(const void * msgin, void * context)
  {
    printf("rclc-callback 0 ");
    Fusion * this_ptr = static_cast<Fusion *>(context);
    if (this_ptr != NULL) {
      this_ptr->subscriptions_[0].cache = static_cast<const message_t *>(msgin);
      printf(" called ");
    }
    printf("\n");
  }

  static void input_callback_1(const void * msgin, void * context)
  {
    uint64_t timestamp = now_as_int();
    printf("rclc-callback 1 ");
    Fusion * this_ptr = (Fusion *) context;
    if (this_ptr != NULL) {
      this_ptr->subscriptions_[1].cache = static_cast<const message_t *>(msgin);

      // only process and publish when we can perform an actual fusion, this means
      // we have received a sample from each subscription
      if ((this_ptr->subscriptions_[0].cache == NULL) ||
        (this_ptr->subscriptions_[1].cache == NULL))
      {
        return;
      }

      auto number_cruncher_result = number_cruncher(this_ptr->number_crunch_limit_);

      uint32_t missed_samples = get_missed_samples_and_update_seq_nr(
        this_ptr->subscriptions_[0].cache,
        this_ptr->subscriptions_[0].sequence_number)
        +
        get_missed_samples_and_update_seq_nr(
        this_ptr->subscriptions_[1].cache,
        this_ptr->subscriptions_[1].sequence_number);
      /*
          this_ptr->pub_msg_.size = 0;
          merge_history_into_sample(this_ptr->pub_msg_, this_ptr->subscriptions_[0].cache);
          merge_history_into_sample(this_ptr->pub_msg_, this_ptr->subscriptions_[1].cache);
          set_sample(
            this_ptr->get_name(), this_ptr->sequence_number_++, missed_samples, timestamp,
            this_ptr->pub_msg_);
          this_ptr->pub_msg_.data[0] = number_cruncher_result;
          rcl_publish(&this_ptr->publisher_, &this_ptr->pub_msg_, NULL);
      */
      this_ptr->output_message_.size = 0;
      merge_history_into_sample(this_ptr->output_message_, this_ptr->subscriptions_[0].cache);
      merge_history_into_sample(this_ptr->output_message_, this_ptr->subscriptions_[1].cache);
      set_sample(
        this_ptr->get_name(), this_ptr->sequence_number_++, missed_samples, timestamp,
        this_ptr->output_message_);
      this_ptr->output_message_.data[0] = number_cruncher_result;
      rcl_publish(&this_ptr->publisher_, &this_ptr->output_message_, NULL);

      this_ptr->subscriptions_[0].cache = NULL;
      this_ptr->subscriptions_[1].cache = NULL;

      printf(" called");
    }
    printf("\n");
  }

private:
  struct subscription_t
  {
    rcl_subscription_t subscription;
    uint32_t sequence_number = 0;
    const message_t * cache;
  };

  subscription_t subscriptions_[2];

  const rosidl_message_type_support_t * type_support_;
  MESSAGE_T_FULL_NAME msg_0_;
  MESSAGE_T_FULL_NAME msg_1_;
  // MESSAGE_T_FULL_NAME pub_msg_;
  message_t output_message_;
  rcl_publisher_t publisher_;

  uint64_t number_crunch_limit_;
  uint32_t sequence_number_ = 0;
};

}  // namespace rclc_system
}  // namespace nodes
#endif  // REFERENCE_SYSTEM__NODES__RCLC__FUSION_HPP_
