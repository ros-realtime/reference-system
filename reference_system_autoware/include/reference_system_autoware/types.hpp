#pragma once

#include "rclcpp/rclcpp.hpp"
#include "reference_system_autoware/msg/message.hpp"
#include "std_msgs/msg/string.hpp"

using message_t = reference_system_autoware::msg::Message1kb;
using publisher_t = rclcpp::Publisher<message_t>::SharedPtr;
using subscription_t = rclcpp::Subscription<message_t>::SharedPtr;
