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
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "reference_system/system/type/rclcpp_system.hpp"

#include "autoware_reference_system/autoware_system_builder.hpp"
#include "autoware_reference_system/system/timing/benchmark.hpp"
#include "autoware_reference_system/system/timing/default.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::vector<std::shared_ptr<RclcppSystem::NodeBaseType>> nodes;
  if (argc == 2 && strncmp(argv[1], "benchmark", 9) == 0) {
    nodes =
      create_autoware_nodes<RclcppSystem, nodes::timing::BenchmarkCPUUsage>();
  } else if (argc == 2 && strncmp(argv[1], "quietbenchmark", 14) == 0) {
    set_benchmark_mode(true);
    nodes =
      create_autoware_nodes<RclcppSystem, nodes::timing::BenchmarkCPUUsage>();
  } else {
    nodes = create_autoware_nodes<RclcppSystem, nodes::timing::Default>();
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  for (auto & node : nodes) {
    executor.add_node(node);
  }
  executor.spin();

  nodes.clear();
  rclcpp::shutdown();

  return 0;
}
