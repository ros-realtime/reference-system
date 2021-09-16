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

  // sensors
  static constexpr time_t FRONT_LIDAR_DRIVER = milliseconds(0);
  static constexpr time_t REAR_LIDAR_DRIVER = milliseconds(0);
  static constexpr time_t POINT_CLOUD_MAP = milliseconds(0);
  static constexpr time_t RVIZ2 = milliseconds(0);
  static constexpr time_t LANELET2MAP = milliseconds(0);

  // processing
  static constexpr time_t POINTS_TRANSFORMER_FRONT = milliseconds(0);
  static constexpr time_t POINTS_TRANSFORMER_REAR = milliseconds(0);
  static constexpr time_t VOXEL_GRID_DOWNSAMPLER = milliseconds(0);
  static constexpr time_t POINT_CLOUD_MAP_LOADER = milliseconds(0);
  static constexpr time_t RAY_GROUND_FILTER = milliseconds(0);
  static constexpr time_t EUCLIDEAN_CLUSTER_DETECTOR = milliseconds(0);
  static constexpr time_t OBJECT_COLLISION_ESTIMATOR = milliseconds(0);
  static constexpr time_t MPC_CONTROLLER = milliseconds(0);
  static constexpr time_t PARKING_PLANNER = milliseconds(0);
  static constexpr time_t LANE_PLANNER = milliseconds(0);

  // fusion
  static constexpr time_t POINT_CLOUD_FUSION = milliseconds(0);
  static constexpr time_t NDT_LOCALIZER = milliseconds(0);
  static constexpr time_t VEHICLE_INTERFACE = milliseconds(0);
  static constexpr time_t LANELET_2_GLOBAL_PLANNER = milliseconds(0);
  static constexpr time_t LANELET_2_MAP_LOADER = milliseconds(0);

  // reactor
  static constexpr time_t BEHAVIOR_PLANNER = milliseconds(0);
};

constexpr BenchmarkThroughput::time_t BenchmarkThroughput::FRONT_LIDAR_DRIVER;
constexpr BenchmarkThroughput::time_t BenchmarkThroughput::REAR_LIDAR_DRIVER;
constexpr BenchmarkThroughput::time_t BenchmarkThroughput::POINT_CLOUD_MAP;
constexpr BenchmarkThroughput::time_t BenchmarkThroughput::RVIZ2;
constexpr BenchmarkThroughput::time_t BenchmarkThroughput::LANELET2MAP;
constexpr BenchmarkThroughput::time_t
BenchmarkThroughput::POINTS_TRANSFORMER_FRONT;
constexpr BenchmarkThroughput::time_t
BenchmarkThroughput::POINTS_TRANSFORMER_REAR;
constexpr BenchmarkThroughput::time_t
BenchmarkThroughput::VOXEL_GRID_DOWNSAMPLER;
constexpr BenchmarkThroughput::time_t
BenchmarkThroughput::POINT_CLOUD_MAP_LOADER;
constexpr BenchmarkThroughput::time_t BenchmarkThroughput::RAY_GROUND_FILTER;
constexpr BenchmarkThroughput::time_t
BenchmarkThroughput::EUCLIDEAN_CLUSTER_DETECTOR;
constexpr BenchmarkThroughput::time_t
BenchmarkThroughput::OBJECT_COLLISION_ESTIMATOR;
constexpr BenchmarkThroughput::time_t BenchmarkThroughput::MPC_CONTROLLER;
constexpr BenchmarkThroughput::time_t BenchmarkThroughput::PARKING_PLANNER;
constexpr BenchmarkThroughput::time_t BenchmarkThroughput::LANE_PLANNER;
constexpr BenchmarkThroughput::time_t BenchmarkThroughput::POINT_CLOUD_FUSION;
constexpr BenchmarkThroughput::time_t BenchmarkThroughput::NDT_LOCALIZER;
constexpr BenchmarkThroughput::time_t BenchmarkThroughput::VEHICLE_INTERFACE;
constexpr BenchmarkThroughput::time_t
BenchmarkThroughput::LANELET_2_GLOBAL_PLANNER;
constexpr BenchmarkThroughput::time_t BenchmarkThroughput::LANELET_2_MAP_LOADER;
constexpr BenchmarkThroughput::time_t BenchmarkThroughput::BEHAVIOR_PLANNER;

struct BenchmarkCPUUsage
{
  using time_t = std::chrono::nanoseconds;
  using milliseconds = std::chrono::milliseconds;

  // sensors
  static constexpr time_t FRONT_LIDAR_DRIVER = milliseconds(50);
  static constexpr time_t REAR_LIDAR_DRIVER = milliseconds(50);
  static constexpr time_t POINT_CLOUD_MAP = milliseconds(50);
  static constexpr time_t RVIZ2 = milliseconds(50);
  static constexpr time_t LANELET2MAP = milliseconds(50);

  // processing
  static constexpr time_t POINTS_TRANSFORMER_FRONT = milliseconds(0);
  static constexpr time_t POINTS_TRANSFORMER_REAR = milliseconds(0);
  static constexpr time_t VOXEL_GRID_DOWNSAMPLER = milliseconds(0);
  static constexpr time_t POINT_CLOUD_MAP_LOADER = milliseconds(0);
  static constexpr time_t RAY_GROUND_FILTER = milliseconds(0);
  static constexpr time_t EUCLIDEAN_CLUSTER_DETECTOR = milliseconds(0);
  static constexpr time_t OBJECT_COLLISION_ESTIMATOR = milliseconds(0);
  static constexpr time_t MPC_CONTROLLER = milliseconds(0);
  static constexpr time_t PARKING_PLANNER = milliseconds(0);
  static constexpr time_t LANE_PLANNER = milliseconds(0);

  // fusion
  static constexpr time_t POINT_CLOUD_FUSION = milliseconds(0);
  static constexpr time_t NDT_LOCALIZER = milliseconds(0);
  static constexpr time_t VEHICLE_INTERFACE = milliseconds(0);
  static constexpr time_t LANELET_2_GLOBAL_PLANNER = milliseconds(0);
  static constexpr time_t LANELET_2_MAP_LOADER = milliseconds(0);

  // reactor
  static constexpr time_t BEHAVIOR_PLANNER = milliseconds(0);
};

constexpr BenchmarkCPUUsage::time_t BenchmarkCPUUsage::FRONT_LIDAR_DRIVER;
constexpr BenchmarkCPUUsage::time_t BenchmarkCPUUsage::REAR_LIDAR_DRIVER;
constexpr BenchmarkCPUUsage::time_t BenchmarkCPUUsage::POINT_CLOUD_MAP;
constexpr BenchmarkCPUUsage::time_t BenchmarkCPUUsage::RVIZ2;
constexpr BenchmarkCPUUsage::time_t BenchmarkCPUUsage::LANELET2MAP;
constexpr BenchmarkCPUUsage::time_t BenchmarkCPUUsage::POINTS_TRANSFORMER_FRONT;
constexpr BenchmarkCPUUsage::time_t BenchmarkCPUUsage::POINTS_TRANSFORMER_REAR;
constexpr BenchmarkCPUUsage::time_t BenchmarkCPUUsage::VOXEL_GRID_DOWNSAMPLER;
constexpr BenchmarkCPUUsage::time_t BenchmarkCPUUsage::POINT_CLOUD_MAP_LOADER;
constexpr BenchmarkCPUUsage::time_t BenchmarkCPUUsage::RAY_GROUND_FILTER;
constexpr BenchmarkCPUUsage::time_t
BenchmarkCPUUsage::EUCLIDEAN_CLUSTER_DETECTOR;
constexpr BenchmarkCPUUsage::time_t
BenchmarkCPUUsage::OBJECT_COLLISION_ESTIMATOR;
constexpr BenchmarkCPUUsage::time_t BenchmarkCPUUsage::MPC_CONTROLLER;
constexpr BenchmarkCPUUsage::time_t BenchmarkCPUUsage::PARKING_PLANNER;
constexpr BenchmarkCPUUsage::time_t BenchmarkCPUUsage::LANE_PLANNER;
constexpr BenchmarkCPUUsage::time_t BenchmarkCPUUsage::POINT_CLOUD_FUSION;
constexpr BenchmarkCPUUsage::time_t BenchmarkCPUUsage::NDT_LOCALIZER;
constexpr BenchmarkCPUUsage::time_t BenchmarkCPUUsage::VEHICLE_INTERFACE;
constexpr BenchmarkCPUUsage::time_t BenchmarkCPUUsage::LANELET_2_GLOBAL_PLANNER;
constexpr BenchmarkCPUUsage::time_t BenchmarkCPUUsage::LANELET_2_MAP_LOADER;
constexpr BenchmarkCPUUsage::time_t BenchmarkCPUUsage::BEHAVIOR_PLANNER;

}  // namespace timing
}  // namespace nodes
#endif  // AUTOWARE_REFERENCE_SYSTEM__SYSTEM__TIMING__BENCHMARK_HPP_
