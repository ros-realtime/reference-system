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
#ifndef AUTOWARE_REFERENCE_SYSTEM__SYSTEM__TIMING__DEFAULT_HPP_
#define AUTOWARE_REFERENCE_SYSTEM__SYSTEM__TIMING__DEFAULT_HPP_
#include <chrono>

namespace nodes
{
namespace timing
{

struct Default
{
  using time_t = std::chrono::nanoseconds;
  using milliseconds = std::chrono::milliseconds;

  // sensors
  static constexpr time_t FRONT_LIDAR_DRIVER = milliseconds(100);
  static constexpr time_t REAR_LIDAR_DRIVER = milliseconds(100);
  static constexpr time_t POINT_CLOUD_MAP = milliseconds(120);
  static constexpr time_t VISUALIZER = milliseconds(60);
  static constexpr time_t LANELET2MAP = milliseconds(100);
  static constexpr time_t EUCLIDEAN_CLUSTER_SETTINGS = milliseconds(1000);

  // the following values are used as the number_cruncher_limit
  // to search for primes up to starting at 3
  // for your platform, run the `number_cruncher_benchmark` executable
  // to figure out what values to place here corresponding to the run_time
  // you would like to run each node for
  // processing
  static constexpr uint64_t POINTS_TRANSFORMER_FRONT = 65536;
  static constexpr uint64_t POINTS_TRANSFORMER_REAR = 65536;
  static constexpr uint64_t VOXEL_GRID_DOWNSAMPLER = 65536;
  static constexpr uint64_t POINT_CLOUD_MAP_LOADER = 65536;
  static constexpr uint64_t RAY_GROUND_FILTER = 65536;
  static constexpr uint64_t EUCLIDEAN_CLUSTER_DETECTOR = 65536;
  static constexpr uint64_t EUCLIDEAN_INTERSECTION = 65536;
  static constexpr uint64_t OBJECT_COLLISION_ESTIMATOR = 65536;
  static constexpr uint64_t MPC_CONTROLLER = 65536;
  static constexpr uint64_t PARKING_PLANNER = 65536;
  static constexpr uint64_t LANE_PLANNER = 65536;

  // fusion
  static constexpr uint64_t POINT_CLOUD_FUSION = 65536;
  static constexpr uint64_t NDT_LOCALIZER = 65536;
  static constexpr uint64_t VEHICLE_INTERFACE = 65536;
  static constexpr uint64_t LANELET_2_GLOBAL_PLANNER = 65536;
  static constexpr uint64_t LANELET_2_MAP_LOADER = 65536;

  // cyclic
  static constexpr uint64_t BEHAVIOR_PLANNER = 65536;
  static constexpr time_t BEHAVIOR_PLANNER_CYCLE = milliseconds(100);
};

constexpr Default::time_t Default::FRONT_LIDAR_DRIVER;
constexpr Default::time_t Default::REAR_LIDAR_DRIVER;
constexpr Default::time_t Default::POINT_CLOUD_MAP;
constexpr Default::time_t Default::VISUALIZER;
constexpr Default::time_t Default::LANELET2MAP;
constexpr Default::time_t Default::EUCLIDEAN_CLUSTER_SETTINGS;
constexpr uint64_t Default::POINTS_TRANSFORMER_FRONT;
constexpr uint64_t Default::POINTS_TRANSFORMER_REAR;
constexpr uint64_t Default::VOXEL_GRID_DOWNSAMPLER;
constexpr uint64_t Default::POINT_CLOUD_MAP_LOADER;
constexpr uint64_t Default::RAY_GROUND_FILTER;
constexpr uint64_t Default::EUCLIDEAN_CLUSTER_DETECTOR;
constexpr uint64_t Default::EUCLIDEAN_INTERSECTION;
constexpr uint64_t Default::OBJECT_COLLISION_ESTIMATOR;
constexpr uint64_t Default::MPC_CONTROLLER;
constexpr uint64_t Default::PARKING_PLANNER;
constexpr uint64_t Default::LANE_PLANNER;
constexpr uint64_t Default::POINT_CLOUD_FUSION;
constexpr uint64_t Default::NDT_LOCALIZER;
constexpr uint64_t Default::VEHICLE_INTERFACE;
constexpr uint64_t Default::LANELET_2_GLOBAL_PLANNER;
constexpr uint64_t Default::LANELET_2_MAP_LOADER;
constexpr uint64_t Default::BEHAVIOR_PLANNER;
constexpr Default::time_t Default::BEHAVIOR_PLANNER_CYCLE;

}  // namespace timing
}  // namespace nodes
#endif  // AUTOWARE_REFERENCE_SYSTEM__SYSTEM__TIMING__DEFAULT_HPP_
