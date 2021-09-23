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
  static constexpr time_t POINT_CLOUD_MAP = milliseconds(100);
  static constexpr time_t VISUALIZER = milliseconds(100);
  static constexpr time_t LANELET2MAP = milliseconds(100);

  // processing
  static constexpr time_t POINTS_TRANSFORMER_FRONT = milliseconds(50);
  static constexpr time_t POINTS_TRANSFORMER_REAR = milliseconds(50);
  static constexpr time_t VOXEL_GRID_DOWNSAMPLER = milliseconds(50);
  static constexpr time_t POINT_CLOUD_MAP_LOADER = milliseconds(50);
  static constexpr time_t RAY_GROUND_FILTER = milliseconds(50);
  static constexpr time_t EUCLIDEAN_CLUSTER_DETECTOR = milliseconds(50);
  static constexpr time_t OBJECT_COLLISION_ESTIMATOR = milliseconds(50);
  static constexpr time_t MPC_CONTROLLER = milliseconds(50);
  static constexpr time_t PARKING_PLANNER = milliseconds(50);
  static constexpr time_t LANE_PLANNER = milliseconds(50);

  // fusion
  static constexpr time_t POINT_CLOUD_FUSION = milliseconds(25);
  static constexpr time_t NDT_LOCALIZER = milliseconds(25);
  static constexpr time_t VEHICLE_INTERFACE = milliseconds(25);
  static constexpr time_t LANELET_2_GLOBAL_PLANNER = milliseconds(25);
  static constexpr time_t LANELET_2_MAP_LOADER = milliseconds(25);

  static constexpr time_t POINT_CLOUD_FUSION_MAX_INPUT_TIME_DIFF = milliseconds(125);
  static constexpr time_t NDT_LOCALIZER_MAX_INPUT_TIME_DIFF = milliseconds(125);
  static constexpr time_t VEHICLE_INTERFACE_MAX_INPUT_TIME_DIFF = milliseconds(125);
  static constexpr time_t LANELET_2_GLOBAL_PLANNER_MAX_INPUT_TIME_DIFF = milliseconds(125);
  static constexpr time_t LANELET_2_MAP_LOADER_MAX_INPUT_TIME_DIFF = milliseconds(125);

  // reactor
  static constexpr time_t BEHAVIOR_PLANNER = milliseconds(1);
};

constexpr Default::time_t Default::FRONT_LIDAR_DRIVER;
constexpr Default::time_t Default::REAR_LIDAR_DRIVER;
constexpr Default::time_t Default::POINT_CLOUD_MAP;
constexpr Default::time_t Default::VISUALIZER;
constexpr Default::time_t Default::LANELET2MAP;
constexpr Default::time_t Default::POINTS_TRANSFORMER_FRONT;
constexpr Default::time_t Default::POINTS_TRANSFORMER_REAR;
constexpr Default::time_t Default::VOXEL_GRID_DOWNSAMPLER;
constexpr Default::time_t Default::POINT_CLOUD_MAP_LOADER;
constexpr Default::time_t Default::RAY_GROUND_FILTER;
constexpr Default::time_t Default::EUCLIDEAN_CLUSTER_DETECTOR;
constexpr Default::time_t Default::OBJECT_COLLISION_ESTIMATOR;
constexpr Default::time_t Default::MPC_CONTROLLER;
constexpr Default::time_t Default::PARKING_PLANNER;
constexpr Default::time_t Default::LANE_PLANNER;
constexpr Default::time_t Default::POINT_CLOUD_FUSION;
constexpr Default::time_t Default::NDT_LOCALIZER;
constexpr Default::time_t Default::VEHICLE_INTERFACE;
constexpr Default::time_t Default::LANELET_2_GLOBAL_PLANNER;
constexpr Default::time_t Default::LANELET_2_MAP_LOADER;
constexpr Default::time_t Default::POINT_CLOUD_FUSION_MAX_INPUT_TIME_DIFF;
constexpr Default::time_t Default::NDT_LOCALIZER_MAX_INPUT_TIME_DIFF;
constexpr Default::time_t Default::VEHICLE_INTERFACE_MAX_INPUT_TIME_DIFF;
constexpr Default::time_t Default::LANELET_2_GLOBAL_PLANNER_MAX_INPUT_TIME_DIFF;
constexpr Default::time_t Default::LANELET_2_MAP_LOADER_MAX_INPUT_TIME_DIFF;
constexpr Default::time_t Default::BEHAVIOR_PLANNER;

}  // namespace timing
}  // namespace nodes
#endif  // AUTOWARE_REFERENCE_SYSTEM__SYSTEM__TIMING__DEFAULT_HPP_
