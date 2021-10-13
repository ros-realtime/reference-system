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
#include <algorithm>
#include <string>
#include <iostream>

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
  //using TimeConfig = nodes::timing::BenchmarkCPUUsage;
  // set_benchmark_mode(true);

  auto nodes = create_autoware_nodes<RclcppSystem, TimeConfig>();

  rclcpp::executors::SingleThreadedExecutor executor1, executor2, executor3, executor4;
  
  executor1.enable_callback_priority();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PiCAS priority-based callback scheduling: %s", executor1.callback_priority_enabled ? "Enabled" : "Disabled");
  executor1.set_executor_priority_cpu(90, 0);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PiCAS executor 1's rt-priority %d and CPU %d", executor1.executor_priority, executor1.executor_cpu);

  executor2.enable_callback_priority();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PiCAS priority-based callback scheduling: %s", executor2.callback_priority_enabled ? "Enabled" : "Disabled");
  executor2.set_executor_priority_cpu(89, 1);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PiCAS executor 2's rt-priority %d and CPU %d", executor2.executor_priority, executor2.executor_cpu);

  executor3.enable_callback_priority();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PiCAS priority-based callback scheduling: %s", executor3.callback_priority_enabled ? "Enabled" : "Disabled");
  executor3.set_executor_priority_cpu(88, 2);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PiCAS executor 3's rt-priority %d and CPU %d", executor3.executor_priority, executor3.executor_cpu);

  executor4.enable_callback_priority();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PiCAS priority-based callback scheduling: %s", executor4.callback_priority_enabled ? "Enabled" : "Disabled");
  executor4.set_executor_priority_cpu(87, 3);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PiCAS executor 4's rt-priority %d and CPU %d", executor4.executor_priority, executor4.executor_cpu);

  std::vector<std::string> executor1_nodes {"VehicleDBWSystem", "VehicleInterface", "MPCController", "BehaviorPlanner"};
  std::vector<std::string> executor2_nodes {"FrontLidarDriver", "RearLidarDriver", "PointsTransformerFront",
  "PointsTransformerRear", "PointCloudFusion", "RayGroundFilter", "ObjectCollisionEstimator"};
  std::vector<std::string> executor3_nodes {"EuclideanClusterDetector", "EuclideanClusterSettings", "IntersectionOutput"};
  std::vector<std::string> executor4_nodes {"PointCloudMap", "PointCloudMapLoader", "VoxelGridDownsampler", "NDTLocalizer",
  "Visualizer", "Lanelet2Map", "LanePlanner", "ParkingPlanner", "Lanelet2MapLoader", "Lanelet2GlobalPlanner"};
  
  for (auto & node : nodes) {
    if (std::count(executor1_nodes.begin(), executor1_nodes.end(), node->get_name())) {
      executor1.add_node(node);
      //std::cout << "exe1 : " << node->get_name() << std::endl;
    } else if (std::count(executor2_nodes.begin(), executor2_nodes.end(), node->get_name())) {
      executor2.add_node(node);
      //std::cout << "exe2 : " << node->get_name() << std::endl;
    } else if (std::count(executor3_nodes.begin(), executor3_nodes.end(), node->get_name())) {
      executor3.add_node(node);
      //std::cout << "exe3 : " << node->get_name() << std::endl;
    } else if (std::count(executor4_nodes.begin(), executor4_nodes.end(), node->get_name())) {
      executor4.add_node(node);
      //std::cout << "exe4 : " << node->get_name() << std::endl;
    }
  }
  
  std::thread spinThread1(&rclcpp::executors::SingleThreadedExecutor::spin_rt, &executor1);
  std::thread spinThread2(&rclcpp::executors::SingleThreadedExecutor::spin_rt, &executor2);
  std::thread spinThread3(&rclcpp::executors::SingleThreadedExecutor::spin_rt, &executor3);
  std::thread spinThread4(&rclcpp::executors::SingleThreadedExecutor::spin_rt, &executor4);
  spinThread1.join();
  spinThread2.join();
  spinThread3.join();
  spinThread4.join();
  
  nodes.clear();

  rclcpp::shutdown();

  return 0;
}

