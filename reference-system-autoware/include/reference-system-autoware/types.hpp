#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using message_t = std_msgs::msg::String;
using publisher_t = rclcpp::Publisher<message_t>::SharedPtr;
