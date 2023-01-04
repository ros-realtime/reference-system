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

#include "rclcpp/rclcpp.hpp"
#include "reference_system/system/type/rclcpp_system.hpp"

#include "autoware_reference_system/autoware_system_builder.hpp"
#include "autoware_reference_system/system/timing/benchmark.hpp"
#include "autoware_reference_system/system/timing/default.hpp"

#include <kpsr_ros2_executor/streaming_executor_factory.hpp>
#include "config.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  using TimeConfig = nodes::timing::Default;
  // uncomment for benchmarking
  // using TimeConfig = nodes::timing::BenchmarkCPUUsage;
  // set_benchmark_mode(true);

  auto nodes = create_autoware_nodes<RclcppSystem, TimeConfig>();

  const std::string jsonFileName = std::string(CONF_DATA) + std::string("/streaming_conf.json");
  rclcpp::Executor::SharedPtr executor =  kpsr::ros2::StreamingExecutorFactory::createExecutor(jsonFileName);

  for (auto & node : nodes) {
    executor->add_node(node);
  }
  executor->spin();

  nodes.clear();
  rclcpp::shutdown();

  return 0;
}
