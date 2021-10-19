// Copyright (c) 2021 by Robert Bosch GmbH
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
#ifndef AUTOWARE_REFERENCE_SYSTEM__AUTOWARE_SYSTEM_BUILDER_UTILS_HPP_
#define AUTOWARE_REFERENCE_SYSTEM__AUTOWARE_SYSTEM_BUILDER_UTILS_HPP_
#include <unistd.h>
#include <sys/time.h>
#include <sys/resource.h>

#include <algorithm>
#include <atomic>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

/// From the given container of pointers to Node instances, the one with the
/// given name is returned. If no such node exists, the function fails with
/// `RCLCPP_FATAL`.
template<typename NODES>
std::shared_ptr<rclcpp::Node> find_node_or_fail(
  const NODES & nodes, const std::string & name)
{
  static_assert(std::is_same<typename NODES::value_type, std::shared_ptr<rclcpp::Node>>::value);

  for (auto & node : nodes) {
    if (name == node->get_name()) {
      return node;
    }
  }
  RCLCPP_FATAL(rclcpp::get_logger("main"), "Node '%s' was not found!", name.c_str());
  std::exit(1);
}

/// Adds those Nodes from `nodes` to the given Executor whose names appear in `names`.
/// If a name cannot be found or if a node has been already associated with an Executor, the
/// function fails with `RCLCPP_FATAL`.
template<typename NODES, typename NAMES>
void add_nodes_to_executor_by_names(
  rclcpp::Executor * executor, const NODES & nodes, const NAMES & names)
{
  static_assert(std::is_same<typename NODES::value_type, std::shared_ptr<rclcpp::Node>>::value);
  static_assert(std::is_same<typename NAMES::value_type, std::string>::value);

  for (const std::string & name : names) {
    bool was_added = false;
    for (auto & node : nodes) {
      if (name == node->get_name()) {
        if (node->get_node_base_interface()->get_associated_with_executor_atomic().load()) {
          RCLCPP_FATAL(
            rclcpp::get_logger(
              "main"), "Node '%s' is already associated with an executor!", name.c_str());
          std::exit(0);
        }
        executor->add_node(node);
        was_added = true;
      }
    }
    if (!was_added) {
      RCLCPP_FATAL(rclcpp::get_logger("main"), "Node with name '%s' not found!", name.c_str());
      std::exit(0);
    }
  }
}

/// Associated those nodes from `nodes` with the given Executor that have not been associated to
/// an Executor previously.
template<typename NODES>
void add_unassociated_nodes_to_executor(
  rclcpp::Executor * executor,
  const NODES & nodes)
{
  static_assert(std::is_same<typename NODES::value_type, std::shared_ptr<rclcpp::Node>>::value);

  for (auto & node : nodes) {
    if (!node->get_node_base_interface()->get_associated_with_executor_atomic().load()) {
      executor->add_node(node);
    }
  }
}

/// Changes the nice level of the current thread. If the operation fails, the function fails
/// with `RCLCPP_FATAL`.
void set_nice_level_of_current_thread(const int level)
{
  int rval = setpriority(PRIO_PROCESS, gettid(), level);
  if (rval != 0) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "Could not change priority of thread!");
    rclcpp::shutdown();
    std::exit(1);
  }
}

#endif  // AUTOWARE_REFERENCE_SYSTEM__AUTOWARE_SYSTEM_BUILDER_UTILS_HPP_
