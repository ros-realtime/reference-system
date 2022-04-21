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

#ifndef AUTOWARE_REFERENCE_SYSTEM__PRIORITIES_HPP_
#define AUTOWARE_REFERENCE_SYSTEM__PRIORITIES_HPP_

#include <sched.h>
#include <string>
#include <set>
#include <unordered_set>


extern const std::set<std::string> hotpath_nodes;
extern const std::set<std::string> planner_nodes;

constexpr int hotpath_prio = 1;
#define HOTPATH_AFFINITY {1, 2, 3}


constexpr int planner_prio = 30;
#define PLANNER_AFFINITY {0}

#endif  // AUTOWARE_REFERENCE_SYSTEM__PRIORITIES_HPP_
