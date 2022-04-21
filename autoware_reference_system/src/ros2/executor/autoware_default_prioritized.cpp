// Copyright 2021 Robert Bosch GmbH
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
#include <unordered_set>
#include <unordered_map>
#include <set>

#include "rclcpp/rclcpp.hpp"

#include "reference_system/system/type/rclcpp_system.hpp"

#include "autoware_reference_system/autoware_system_builder.hpp"
#include "autoware_reference_system/system/timing/benchmark.hpp"
#include "autoware_reference_system/system/timing/default.hpp"
#include "autoware_reference_system/priorities.hpp"

void set_rt_properties(int prio, const std::unordered_set<size_t> & affinity)
{
  struct sched_param sched_param = { 0 };
  sched_param.sched_priority = prio;
  sched_setscheduler(0, SCHED_RR, &sched_param);

  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  for (const auto cpu : affinity) {
    CPU_SET(cpu, &cpuset);
  }
  sched_setaffinity(0, sizeof(cpuset), &cpuset);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  using TimeConfig = nodes::timing::Default;
  // uncomment for benchmarking
  // using TimeConfig = nodes::timing::BenchmarkCPUUsage;
  // set_benchmark_mode(true);

  auto nodes_vec = create_autoware_nodes<RclcppSystem, TimeConfig>();
  using NodeMap = std::unordered_map<std::string,
      std::shared_ptr<RclcppSystem::NodeBaseType>>;

  NodeMap nodes;
  for (const auto & node : nodes_vec) {
    nodes.emplace(node->get_name(), node);
    std::cout << node->get_name() << "\n";
  }

  rclcpp::executors::SingleThreadedExecutor
    front_exe,
    rear_exe,
    fusion_exe,
    planner_exe,
    other_exe;

  std::set<std::string> front_nodes = {"FrontLidarDriver", "PointsTransformerFront"};
  std::set<std::string> rear_nodes = {"RearLidarDriver", "PointsTransformerRear"};
  std::set<std::string> fusion_nodes = {"PointCloudFusion",
    "RayGroundFilter",
    "EuclideanClusterDetector",
    "ObjectCollisionEstimator"};
  std::set<std::string> planner_nodes = {"BehaviorPlanner"};
  std::set<std::string> other_nodes = {"PointCloudMap",
    "Visualizer",
    "Lanelet2Map",
    "EuclideanClusterSettings",
    "PointCloudMapLoader",
    "MPCController",
    "VehicleInterface",
    "VehicleDBWSystem",
    "NDTLocalizer",
    "Lanelet2GlobalPlanner",
    "Lanelet2MapLoader",
    "ParkingPlanner",
    "LanePlanner",
    "IntersectionOutput",
    "VoxelGridDownsampler"};

  for (const auto & node : front_nodes) {
    front_exe.add_node(nodes.at(node));
  }
  for (const auto & node : rear_nodes) {
    rear_exe.add_node(nodes.at(node));
  }
  for (const auto & node : fusion_nodes) {
    std::cout << node << "\n";
    fusion_exe.add_node(nodes.at(node));
  }
  for (const auto & node : planner_nodes) {
    planner_exe.add_node(nodes.at(node));
  }
  for (const auto & node : other_nodes) {
    other_exe.add_node(nodes.at(node));
  }

  std::thread front_thread {[&]() {
      set_rt_properties(hotpath_prio, HOTPATH_AFFINITY);
      front_exe.spin();
    }};
  std::thread rear_thread {[&]() {
      set_rt_properties(hotpath_prio, HOTPATH_AFFINITY);
      rear_exe.spin();
    }};
  std::thread fusion_thread {[&]() {
      set_rt_properties(hotpath_prio, HOTPATH_AFFINITY);
      fusion_exe.spin();
    }};
  std::thread planner_thread {[&]() {
      set_rt_properties(planner_prio, PLANNER_AFFINITY);
      planner_exe.spin();
    }};
  std::thread other_thread {[&]() {
      other_exe.spin();
    }};

  front_thread.join();
  rear_thread.join();
  fusion_thread.join();
  planner_thread.join();

  rclcpp::shutdown();
}
