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
#ifndef AUTOWARE_REFERENCE_SYSTEM__SYSTEM__TIMING__BENCHMARK_HPP_
#define AUTOWARE_REFERENCE_SYSTEM__SYSTEM__TIMING__BENCHMARK_HPP_
#include <chrono>

namespace nodes
{
namespace timing
{

struct BenchmarkThroughput
{
  using time_t = std::chrono::nanoseconds;
  using milliseconds = std::chrono::milliseconds;
  using seconds = std::chrono::seconds;

  // sensors
  static constexpr time_t FRONT_LIDAR_DRIVER = milliseconds(0);
  static constexpr time_t REAR_LIDAR_DRIVER = milliseconds(0);
  static constexpr time_t POINT_CLOUD_MAP = milliseconds(0);
  static constexpr time_t VISUALIZER = milliseconds(0);
  static constexpr time_t LANELET2MAP = milliseconds(0);

  // the following values are used as the number_cruncher_limit
  // to search for primes up to starting at 3
  // for your platform, run the `number_cruncher_benchmark` executable
  // to figure out what values to place here corresponding to the run_time
  // you would like to run each node for
  // processing
  static constexpr uint64_t POINTS_TRANSFORMER_FRONT = 0;
  static constexpr uint64_t POINTS_TRANSFORMER_REAR = 0;
  static constexpr uint64_t VOXEL_GRID_DOWNSAMPLER = 0;
  static constexpr uint64_t POINT_CLOUD_MAP_LOADER = 0;
  static constexpr uint64_t RAY_GROUND_FILTER = 0;
  static constexpr uint64_t EUCLIDEAN_CLUSTER_DETECTOR = 0;
  static constexpr uint64_t OBJECT_COLLISION_ESTIMATOR = 0;
  static constexpr uint64_t MPC_CONTROLLER = 0;
  static constexpr uint64_t PARKING_PLANNER = 0;
  static constexpr uint64_t LANE_PLANNER = 0;

  // fusion
  static constexpr uint64_t POINT_CLOUD_FUSION = 0;
  static constexpr uint64_t NDT_LOCALIZER = 0;
  static constexpr uint64_t VEHICLE_INTERFACE = 0;
  static constexpr uint64_t LANELET_2_GLOBAL_PLANNER = 0;
  static constexpr uint64_t LANELET_2_MAP_LOADER = 0;

  // cyclic
  static constexpr uint64_t BEHAVIOR_PLANNER = 0;
  static constexpr time_t BEHAVIOR_PLANNER_CYCLE = milliseconds(0);
};

constexpr BenchmarkThroughput::time_t BenchmarkThroughput::FRONT_LIDAR_DRIVER;
constexpr BenchmarkThroughput::time_t BenchmarkThroughput::REAR_LIDAR_DRIVER;
constexpr BenchmarkThroughput::time_t BenchmarkThroughput::POINT_CLOUD_MAP;
constexpr BenchmarkThroughput::time_t BenchmarkThroughput::VISUALIZER;
constexpr BenchmarkThroughput::time_t BenchmarkThroughput::LANELET2MAP;
constexpr uint64_t BenchmarkThroughput::POINTS_TRANSFORMER_FRONT;
constexpr uint64_t BenchmarkThroughput::POINTS_TRANSFORMER_REAR;
constexpr uint64_t BenchmarkThroughput::VOXEL_GRID_DOWNSAMPLER;
constexpr uint64_t BenchmarkThroughput::POINT_CLOUD_MAP_LOADER;
constexpr uint64_t BenchmarkThroughput::RAY_GROUND_FILTER;
constexpr uint64_t BenchmarkThroughput::EUCLIDEAN_CLUSTER_DETECTOR;
constexpr uint64_t BenchmarkThroughput::OBJECT_COLLISION_ESTIMATOR;
constexpr uint64_t BenchmarkThroughput::MPC_CONTROLLER;
constexpr uint64_t BenchmarkThroughput::PARKING_PLANNER;
constexpr uint64_t BenchmarkThroughput::LANE_PLANNER;
constexpr uint64_t BenchmarkThroughput::POINT_CLOUD_FUSION;
constexpr uint64_t BenchmarkThroughput::NDT_LOCALIZER;
constexpr uint64_t BenchmarkThroughput::VEHICLE_INTERFACE;
constexpr uint64_t BenchmarkThroughput::LANELET_2_GLOBAL_PLANNER;
constexpr uint64_t BenchmarkThroughput::LANELET_2_MAP_LOADER;
constexpr uint64_t BenchmarkThroughput::BEHAVIOR_PLANNER;
constexpr BenchmarkThroughput::time_t BenchmarkThroughput::BEHAVIOR_PLANNER_CYCLE;

struct BenchmarkCPUUsage
{
  using time_t = std::chrono::nanoseconds;
  using milliseconds = std::chrono::milliseconds;
  using seconds = std::chrono::seconds;

  // sensors
  static constexpr time_t FRONT_LIDAR_DRIVER = milliseconds(50);
  static constexpr time_t REAR_LIDAR_DRIVER = milliseconds(50);
  static constexpr time_t POINT_CLOUD_MAP = milliseconds(50);
  static constexpr time_t VISUALIZER = milliseconds(50);
  static constexpr time_t LANELET2MAP = milliseconds(50);

  // the following values are used as the number_cruncher_limit
  // to search for primes up to starting at 3
  // for your platform, run the `number_cruncher_benchmark` executable
  // to figure out what values to place here corresponding to the run_time
  // you would like to run each node for
  // processing
  static constexpr uint64_t POINTS_TRANSFORMER_FRONT = 0;
  static constexpr uint64_t POINTS_TRANSFORMER_REAR = 0;
  static constexpr uint64_t VOXEL_GRID_DOWNSAMPLER = 0;
  static constexpr uint64_t POINT_CLOUD_MAP_LOADER = 0;
  static constexpr uint64_t RAY_GROUND_FILTER = 0;
  static constexpr uint64_t EUCLIDEAN_CLUSTER_DETECTOR = 0;
  static constexpr uint64_t OBJECT_COLLISION_ESTIMATOR = 0;
  static constexpr uint64_t MPC_CONTROLLER = 0;
  static constexpr uint64_t PARKING_PLANNER = 0;
  static constexpr uint64_t LANE_PLANNER = 0;

  // fusion
  static constexpr uint64_t POINT_CLOUD_FUSION = 0;
  static constexpr uint64_t NDT_LOCALIZER = 0;
  static constexpr uint64_t VEHICLE_INTERFACE = 0;
  static constexpr uint64_t LANELET_2_GLOBAL_PLANNER = 0;
  static constexpr uint64_t LANELET_2_MAP_LOADER = 0;

  // cyclic
  static constexpr uint64_t BEHAVIOR_PLANNER = 0;
  static constexpr time_t BEHAVIOR_PLANNER_CYCLE = milliseconds(50);
};

constexpr BenchmarkCPUUsage::time_t BenchmarkCPUUsage::FRONT_LIDAR_DRIVER;
constexpr BenchmarkCPUUsage::time_t BenchmarkCPUUsage::REAR_LIDAR_DRIVER;
constexpr BenchmarkCPUUsage::time_t BenchmarkCPUUsage::POINT_CLOUD_MAP;
constexpr BenchmarkCPUUsage::time_t BenchmarkCPUUsage::VISUALIZER;
constexpr BenchmarkCPUUsage::time_t BenchmarkCPUUsage::LANELET2MAP;
constexpr uint64_t BenchmarkCPUUsage::POINTS_TRANSFORMER_FRONT;
constexpr uint64_t BenchmarkCPUUsage::POINTS_TRANSFORMER_REAR;
constexpr uint64_t BenchmarkCPUUsage::VOXEL_GRID_DOWNSAMPLER;
constexpr uint64_t BenchmarkCPUUsage::POINT_CLOUD_MAP_LOADER;
constexpr uint64_t BenchmarkCPUUsage::RAY_GROUND_FILTER;
constexpr uint64_t BenchmarkCPUUsage::EUCLIDEAN_CLUSTER_DETECTOR;
constexpr uint64_t BenchmarkCPUUsage::OBJECT_COLLISION_ESTIMATOR;
constexpr uint64_t BenchmarkCPUUsage::MPC_CONTROLLER;
constexpr uint64_t BenchmarkCPUUsage::PARKING_PLANNER;
constexpr uint64_t BenchmarkCPUUsage::LANE_PLANNER;
constexpr uint64_t BenchmarkCPUUsage::POINT_CLOUD_FUSION;
constexpr uint64_t BenchmarkCPUUsage::NDT_LOCALIZER;
constexpr uint64_t BenchmarkCPUUsage::VEHICLE_INTERFACE;
constexpr uint64_t BenchmarkCPUUsage::LANELET_2_GLOBAL_PLANNER;
constexpr uint64_t BenchmarkCPUUsage::LANELET_2_MAP_LOADER;
constexpr uint64_t BenchmarkCPUUsage::BEHAVIOR_PLANNER;
constexpr BenchmarkCPUUsage::time_t BenchmarkCPUUsage::BEHAVIOR_PLANNER_CYCLE;

}  // namespace timing
}  // namespace nodes
#endif  // AUTOWARE_REFERENCE_SYSTEM__SYSTEM__TIMING__BENCHMARK_HPP_
