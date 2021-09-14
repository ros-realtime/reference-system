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
#ifndef REFERENCE_SYSTEM__SYSTEM__TIMING__DEFAULT_HPP_
#define REFERENCE_SYSTEM__SYSTEM__TIMING__DEFAULT_HPP_
#include <chrono>

namespace nodes
{
namespace timing
{

struct Default
{
  static constexpr std::chrono::nanoseconds SENSOR_CYCLE_TIME =
    std::chrono::milliseconds(200);
  static constexpr std::chrono::nanoseconds PROCESSING_NODE_CRUNCH_TIME =
    std::chrono::milliseconds(719);
  static constexpr std::chrono::nanoseconds FUSION_NODE_CRUNCH_TIME =
    std::chrono::milliseconds(589);
  static constexpr std::chrono::nanoseconds REACTOR_NODE_CRUNCH_TIME =
    std::chrono::milliseconds(0);
};

constexpr std::chrono::nanoseconds Default::SENSOR_CYCLE_TIME;
constexpr std::chrono::nanoseconds Default::PROCESSING_NODE_CRUNCH_TIME;
constexpr std::chrono::nanoseconds Default::FUSION_NODE_CRUNCH_TIME;
constexpr std::chrono::nanoseconds Default::REACTOR_NODE_CRUNCH_TIME;
}  // namespace timing
}  // namespace nodes
#endif  // REFERENCE_SYSTEM__SYSTEM__TIMING__DEFAULT_HPP_
