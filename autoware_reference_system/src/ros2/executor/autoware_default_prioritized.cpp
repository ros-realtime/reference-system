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

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "reference_system/system/systems.hpp"

#include "autoware_reference_system/autoware_system_builder.hpp"
#include "autoware_reference_system/autoware_system_builder_utils.hpp"
#include "autoware_reference_system/system/timing/benchmark.hpp"
#include "autoware_reference_system/system/timing/default.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  using TimeConfig = nodes::timing::Default;
  // uncomment for benchmarking
  // using TimeConfig = nodes::timing::BenchmarkCPUUsage;
  // set_benchmark_mode(true);

  std::vector<std::shared_ptr<rclcpp::Node>> nodes = create_autoware_nodes<RclcppSystem,
      TimeConfig>();
  rclcpp::Logger logger = nodes[0]->get_logger();

  std::vector<std::string> critpath_front_node_names =
  {"FrontLidarDriver", "PointsTransformerFront"};
  std::vector<std::string> critpath_rear_node_names = {"RearLidarDriver", "PointsTransformerRear"};
  std::vector<std::string> critpath_inner_node_names =
  {"PointCloudFusion", "RayGroundFilter", "EuclideanClusterDetector", "ObjectCollisionEstimator"};

  rclcpp::executors::SingleThreadedExecutor critpath_front_executor;
  rclcpp::executors::SingleThreadedExecutor critpath_rear_executor;
  rclcpp::executors::SingleThreadedExecutor critpath_inner_executor;
  rclcpp::executors::SingleThreadedExecutor other_executor;

  add_nodes_to_executor_by_names(&critpath_front_executor, nodes, critpath_front_node_names);
  add_nodes_to_executor_by_names(&critpath_rear_executor, nodes, critpath_rear_node_names);
  add_nodes_to_executor_by_names(&critpath_inner_executor, nodes, critpath_inner_node_names);

  add_unassociated_nodes_to_executor(&other_executor, nodes);

  auto critpath_front_thread = std::thread(
    [&]() {
      set_nice_level_of_current_thread(-5);
      critpath_front_executor.spin();
    });
  auto critpath_rear_thread = std::thread(
    [&]() {
      set_nice_level_of_current_thread(-5);
      critpath_rear_executor.spin();
    });
  auto critpath_inner_thread = std::thread(
    [&]() {
      set_nice_level_of_current_thread(-5);
      critpath_inner_executor.spin();
    });
  auto other_thread = std::thread(
    [&]() {
      set_nice_level_of_current_thread(+5);
      other_executor.spin();
    });

  critpath_front_thread.join();
  critpath_rear_thread.join();
  critpath_inner_thread.join();
  other_thread.join();

  nodes.clear();
  rclcpp::shutdown();

  return 0;
}
