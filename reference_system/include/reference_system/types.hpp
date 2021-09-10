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
#ifndef REFERENCE_SYSTEM__TYPES_HPP_
#define REFERENCE_SYSTEM__TYPES_HPP_
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "reference_interfaces/msg/message4kb.hpp"
#include "std_msgs/msg/string.hpp"

using message_t = reference_interfaces::msg::Message4kb;
using publisher_t = rclcpp::Publisher<message_t>::SharedPtr;
using subscription_t = rclcpp::Subscription<message_t>::SharedPtr;

#endif  // REFERENCE_SYSTEM__TYPES_HPP_
