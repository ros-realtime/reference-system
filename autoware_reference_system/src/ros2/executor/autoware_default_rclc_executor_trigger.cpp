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

#include <stdio.h>
#include <string.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "reference_system/system/systems.hpp"

#include "autoware_reference_system/autoware_system_builder.hpp"
#include "autoware_reference_system/system/timing/benchmark.hpp"
#include "autoware_reference_system/system/timing/default.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  using TimeConfig = nodes::timing::Default;
  // uncomment for benchmarking
  // using TimeConfig = nodes::timing::BenchmarkCPUUsage;
  // set_benchmark_mode(true);

  auto nodes = create_autoware_nodes<RclcTriggerSystem, TimeConfig>();

  rcl_ret_t rc;
  rcl_context_t * context =
    rclcpp::contexts::get_global_default_context()->get_rcl_context().get();
  rcl_allocator_t allocator = rcl_get_default_allocator();

  // rclc-Executor for PointCloudFusion node
  rclc_executor_t rclcExecutorTrigger = rclc_executor_get_zero_initialized_executor();
  unsigned int num_handles_trigger = 2 * 1;  // two subscription callbacks * 1 nodes
  rc = rclc_executor_init(&rclcExecutorTrigger, context, num_handles_trigger, &allocator);
  if (rc != RCL_RET_OK) {printf("Error rclc_executor_init\n");}
  rc = rclc_executor_set_trigger(&rclcExecutorTrigger, rclc_executor_trigger_all, NULL);
  if (rc != RCL_RET_OK) {printf("Error rclc_executor_set_trigger\n");}

  // rclc-Executor for all other Fusion nodes
  rclc_executor_t rclcExecutor = rclc_executor_get_zero_initialized_executor();
  unsigned int num_handles = 2 * 4;  // two subscription callbacks * 4 nodes
  rclc_executor_init(&rclcExecutor, context, num_handles, &allocator);
  if (rc != RCL_RET_OK) {printf("Error rclc_executor_init\n");}

  // rclcpp Executor for all other nodes
  rclcpp::executors::SingleThreadedExecutor executor;

  for (auto & node : nodes) {
    if (strcmp(node->get_name(), "PointCloudFusion") == 0) {
      printf("... add node '%s' to rclc-executor-trigger.\n", node->get_name());
      std::shared_ptr<nodes::rclc_trigger_system::Fusion> fusionNode =
        std::dynamic_pointer_cast<nodes::rclc_trigger_system::Fusion>(node);
      fusionNode->add_to_executor(&rclcExecutorTrigger);
    } else {
      if ( (strcmp(node->get_name(), "NDTLocalizer") == 0) ||
        (strcmp(node->get_name(), "VehicleInterface") == 0) ||
        (strcmp(node->get_name(), "Lanelet2GlobalPlanner") == 0) ||
        (strcmp(node->get_name(), "Lanelet2MapLoader") == 0))
      {
        printf("... add node '%s' to rclc-executor.\n", node->get_name());
        std::shared_ptr<nodes::rclc_trigger_system::Fusion> fusionNode =
          std::dynamic_pointer_cast<nodes::rclc_trigger_system::Fusion>(node);
        fusionNode->add_to_executor(&rclcExecutor);
      } else {
        printf("... add node '%s' to rclcpp-executor.\n", node->get_name());
        executor.add_node(node);
      }
    }
  }

  uint64_t timeout_ns = 10000000;  // 10ms
  while (rclcpp::ok(rclcpp::contexts::get_global_default_context())) {
    executor.spin_some();
    rc = rclc_executor_spin_some(&rclcExecutorTrigger, timeout_ns);
    if (rc != RCL_RET_OK) { printf("Error rclc_executor_spin_some for trigger.\n");}
    rc = rclc_executor_spin_some(&rclcExecutor, timeout_ns);
    if (rc != RCL_RET_OK) { printf("Error rclc_executor_spin_some for default.\n");}
  }

  nodes.clear();
  rclcpp::shutdown();

  return 0;
}
