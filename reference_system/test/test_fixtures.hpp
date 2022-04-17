// Copyright 2022 Apex.AI, Inc.
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
#ifndef TEST_FIXTURES_HPP_
#define TEST_FIXTURES_HPP_

#include <gtest/gtest.h>

#include <memory>

#include "rclcpp/rclcpp.hpp"


class TestNodeGraph : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }
};

using milliseconds = std::chrono::milliseconds;
static constexpr uint64_t CRUNCH = 65536;

template<typename SystemType, typename NodeType, typename SettingsType>
auto create_node(SettingsType settings)
->std::shared_ptr<typename SystemType::NodeBaseType>
{
  auto node =
    std::make_shared<NodeType>(settings);
  return node;
}

#endif  // TEST_FIXTURES_HPP_
