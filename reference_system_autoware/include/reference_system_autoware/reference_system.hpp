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
#ifndef REFERENCE_SYSTEM_AUTOWARE__REFERENCE_SYSTEM_HPP_
#define REFERENCE_SYSTEM_AUTOWARE__REFERENCE_SYSTEM_HPP_
#include <chrono>
#include <vector>
#include <memory>

#include "reference_system_autoware/config.hpp"

using namespace std::chrono_literals;  // NOLINT

std::vector<std::shared_ptr<node::NodeBaseType>>
create_reference_system_nodes() {
  std::vector<std::shared_ptr<node::NodeBaseType>> nodes;

// ignore the warning about designated initializers - they make the code much
// more readable
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"

  // setup communication graph
  // sensor nodes
  constexpr auto CYCLE_TIME = 200ms;
  nodes.emplace_back(std::make_shared<node::Sensor>(
      SensorSettings{.node_name = "FrontLidarDriver",
                     .topic_name = "FrontLidarDriver",
                     .cycle_time = CYCLE_TIME}));

  nodes.emplace_back(std::make_shared<node::Sensor>(
      SensorSettings{.node_name = "RearLidarDriver",
                     .topic_name = "RearLidarDriver",
                     .cycle_time = CYCLE_TIME}));

  nodes.emplace_back(std::make_shared<node::Sensor>(
      SensorSettings{.node_name = "PointCloudMap",
                     .topic_name = "PointCloudMap",
                     .cycle_time = CYCLE_TIME}));

  nodes.emplace_back(std::make_shared<node::Sensor>(SensorSettings{
      .node_name = "rviz2", .topic_name = "rviz2", .cycle_time = CYCLE_TIME}));

  nodes.emplace_back(
      std::make_shared<node::Sensor>(SensorSettings{.node_name = "Lanelet2Map",
                                                    .topic_name = "Lanelet2Map",
                                                    .cycle_time = CYCLE_TIME}));

  // processing nodes
  constexpr auto PROCESSING_TIME = 719ms;
  nodes.emplace_back(std::make_shared<node::Processing>(
      ProcessingSettings{.node_name = "PointsTransformerFront",
                         .input_topic = "FrontLidarDriver",
                         .output_topic = "PointsTransformerFront",
                         .number_crunch_time = PROCESSING_TIME}));

  nodes.emplace_back(std::make_shared<node::Processing>(
      ProcessingSettings{.node_name = "PointsTransformerRear",
                         .input_topic = "RearLidarDriver",
                         .output_topic = "PointsTransformerRear",
                         .number_crunch_time = PROCESSING_TIME}));

  nodes.emplace_back(std::make_shared<node::Processing>(
      ProcessingSettings{.node_name = "VoxelGridDownsampler",
                         .input_topic = "PointCloudFusion",
                         .output_topic = "VoxelGridDownsampler",
                         .number_crunch_time = PROCESSING_TIME}));

  nodes.emplace_back(std::make_shared<node::Processing>(
      ProcessingSettings{.node_name = "PointCloudMapLoader",
                         .input_topic = "PointCloudMap",
                         .output_topic = "PointCloudMapLoader",
                         .number_crunch_time = PROCESSING_TIME}));

  nodes.emplace_back(std::make_shared<node::Processing>(
      ProcessingSettings{.node_name = "RayGroundFilter",
                         .input_topic = "PointCloudFusion",
                         .output_topic = "RayGroundFilter",
                         .number_crunch_time = PROCESSING_TIME}));

  nodes.emplace_back(std::make_shared<node::Processing>(
      ProcessingSettings{.node_name = "EuclideanClusterDetector",
                         .input_topic = "RayGroundFilter",
                         .output_topic = "EuclideanClusterDetector",
                         .number_crunch_time = PROCESSING_TIME}));

  nodes.emplace_back(std::make_shared<node::Processing>(
      ProcessingSettings{.node_name = "ObjectCollisionEstimator",
                         .input_topic = "EuclideanClusterDetector",
                         .output_topic = "ObjectCollisionEstimator",
                         .number_crunch_time = PROCESSING_TIME}));

  nodes.emplace_back(std::make_shared<node::Processing>(
      ProcessingSettings{.node_name = "MPCController",
                         .input_topic = "BehaviorPlanner",
                         .output_topic = "MPCController",
                         .number_crunch_time = PROCESSING_TIME}));

  nodes.emplace_back(std::make_shared<node::Processing>(
      ProcessingSettings{.node_name = "ParkingPlanner",
                         .input_topic = "Lanelet2MapLoader",
                         .output_topic = "ParkingPlanner",
                         .number_crunch_time = PROCESSING_TIME}));

  nodes.emplace_back(std::make_shared<node::Processing>(
      ProcessingSettings{.node_name = "LanePlanner",
                         .input_topic = "Lanelet2MapLoader",
                         .output_topic = "LanePlanner",
                         .number_crunch_time = PROCESSING_TIME}));

  // fusion nodes
  constexpr auto FUSION_TIME = 589ms;
  nodes.emplace_back(std::make_shared<node::Fusion>(
      FusionSettings{.node_name = "PointCloudFusion",
                     .input_0 = "PointsTransformerFront",
                     .input_1 = "PointsTransformerRear",
                     .output_topic = "PointCloudFusion",
                     .number_crunch_time = FUSION_TIME}));

  nodes.emplace_back(std::make_shared<node::Fusion>(
      FusionSettings{.node_name = "NDTLocalizer",
                     .input_0 = "VoxelGridDownsampler",
                     .input_1 = "PointCloudMapLoader",
                     .output_topic = "NDTLocalizer",
                     .number_crunch_time = FUSION_TIME}));

  nodes.emplace_back(std::make_shared<node::Fusion>(
      FusionSettings{.node_name = "VehicleInterface",
                     .input_0 = "MPCController",
                     .input_1 = "BehaviorPlanner",
                     .output_topic = "VehicleInterface",
                     .number_crunch_time = FUSION_TIME}));

  nodes.emplace_back(std::make_shared<node::Fusion>(
      FusionSettings{.node_name = "Lanelet2GlobalPlanner",
                     .input_0 = "rviz2",
                     .input_1 = "NDTLocalizer",
                     .output_topic = "Lanelet2GlobalPlanner",
                     .number_crunch_time = FUSION_TIME}));

  nodes.emplace_back(std::make_shared<node::Fusion>(
      FusionSettings{.node_name = "Lanelet2MapLoader",
                     .input_0 = "Lanelet2Map",
                     .input_1 = "Lanelet2GlobalPlanner",
                     .output_topic = "Lanelet2MapLoader",
                     .number_crunch_time = PROCESSING_TIME}));

  // reactor node
  constexpr auto REACTOR_TIME = 0ms;
  nodes.emplace_back(std::make_shared<node::Reactor>(
      ReactorSettings{.node_name = "BehaviorPlanner",
                      .inputs = {"ObjectCollisionEstimator", "NDTLocalizer",
                                 "Lanelet2GlobalPlanner", "Lanelet2MapLoader",
                                 "ParkingPlanner", "LanePlanner"},
                      .output_topic = "BehaviorPlanner",
                      .number_crunch_time = REACTOR_TIME}));

  // command node
  nodes.emplace_back(std::make_shared<node::Command>(CommandSettings{
      .node_name = "VehicleDBWSystem", .input_topic = "VehicleInterface"}));
#pragma GCC diagnostic pop

  return nodes;
}
#endif  // REFERENCE_SYSTEM_AUTOWARE__REFERENCE_SYSTEM_HPP_
