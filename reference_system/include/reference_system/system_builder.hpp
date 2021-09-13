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
#ifndef REFERENCE_SYSTEM__SYSTEM_BUILDER_HPP_
#define REFERENCE_SYSTEM__SYSTEM_BUILDER_HPP_
#include <chrono>
#include <memory>
#include <vector>

#include "reference_system/system/systems.hpp"
#include "reference_system/nodes/settings.hpp"

using namespace std::chrono_literals;  // NOLINT

template<typename SystemType>
auto create_system_nodes()
->std::vector<std::shared_ptr<typename SystemType::NodeBaseType>>
{
  std::vector<std::shared_ptr<typename SystemType::NodeBaseType>> nodes;

// ignore the warning about designated initializers - they make the code much
// more readable
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"

  // setup communication graph
  // sensor nodes
  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      nodes::SensorSettings{.node_name = "FrontLidarDriver",
        .topic_name = "FrontLidarDriver",
        .cycle_time = nodes::timing::Default::SENSOR_CYCLE_TIME}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      nodes::SensorSettings{.node_name = "RearLidarDriver",
        .topic_name = "RearLidarDriver",
        .cycle_time = nodes::timing::Default::SENSOR_CYCLE_TIME}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      nodes::SensorSettings{.node_name = "PointCloudMap",
        .topic_name = "PointCloudMap",
        .cycle_time = nodes::timing::Default::SENSOR_CYCLE_TIME}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      nodes::SensorSettings{.node_name = "rviz2",
        .topic_name = "rviz2",
        .cycle_time = nodes::timing::Default::SENSOR_CYCLE_TIME}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      nodes::SensorSettings{.node_name = "Lanelet2Map",
        .topic_name = "Lanelet2Map",
        .cycle_time = nodes::timing::Default::SENSOR_CYCLE_TIME}));

  // processing nodes
  nodes.emplace_back(
    std::make_shared<typename SystemType::Processing>(
      nodes::ProcessingSettings{
    .node_name = "PointsTransformerFront",
    .input_topic = "FrontLidarDriver",
    .output_topic = "PointsTransformerFront",
    .number_crunch_time =
    nodes::timing::Default::PROCESSING_NODE_CRUNCH_TIME}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Processing>(
      nodes::ProcessingSettings{
    .node_name = "PointsTransformerRear",
    .input_topic = "RearLidarDriver",
    .output_topic = "PointsTransformerRear",
    .number_crunch_time =
    nodes::timing::Default::PROCESSING_NODE_CRUNCH_TIME}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Processing>(
      nodes::ProcessingSettings{
    .node_name = "VoxelGridDownsampler",
    .input_topic = "PointCloudFusion",
    .output_topic = "VoxelGridDownsampler",
    .number_crunch_time =
    nodes::timing::Default::PROCESSING_NODE_CRUNCH_TIME}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Processing>(
      nodes::ProcessingSettings{
    .node_name = "PointCloudMapLoader",
    .input_topic = "PointCloudMap",
    .output_topic = "PointCloudMapLoader",
    .number_crunch_time =
    nodes::timing::Default::PROCESSING_NODE_CRUNCH_TIME}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Processing>(
      nodes::ProcessingSettings{
    .node_name = "RayGroundFilter",
    .input_topic = "PointCloudFusion",
    .output_topic = "RayGroundFilter",
    .number_crunch_time =
    nodes::timing::Default::PROCESSING_NODE_CRUNCH_TIME}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Processing>(
      nodes::ProcessingSettings{
    .node_name = "EuclideanClusterDetector",
    .input_topic = "RayGroundFilter",
    .output_topic = "EuclideanClusterDetector",
    .number_crunch_time =
    nodes::timing::Default::PROCESSING_NODE_CRUNCH_TIME}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Processing>(
      nodes::ProcessingSettings{
    .node_name = "ObjectCollisionEstimator",
    .input_topic = "EuclideanClusterDetector",
    .output_topic = "ObjectCollisionEstimator",
    .number_crunch_time =
    nodes::timing::Default::PROCESSING_NODE_CRUNCH_TIME}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Processing>(
      nodes::ProcessingSettings{
    .node_name = "MPCController",
    .input_topic = "BehaviorPlanner",
    .output_topic = "MPCController",
    .number_crunch_time =
    nodes::timing::Default::PROCESSING_NODE_CRUNCH_TIME}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Processing>(
      nodes::ProcessingSettings{
    .node_name = "ParkingPlanner",
    .input_topic = "Lanelet2MapLoader",
    .output_topic = "ParkingPlanner",
    .number_crunch_time =
    nodes::timing::Default::PROCESSING_NODE_CRUNCH_TIME}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Processing>(
      nodes::ProcessingSettings{
    .node_name = "LanePlanner",
    .input_topic = "Lanelet2MapLoader",
    .output_topic = "LanePlanner",
    .number_crunch_time =
    nodes::timing::Default::PROCESSING_NODE_CRUNCH_TIME}));

  // fusion nodes
  nodes.emplace_back(
    std::make_shared<typename SystemType::Fusion>(
      nodes::FusionSettings{
    .node_name = "PointCloudFusion",
    .input_0 = "PointsTransformerFront",
    .input_1 = "PointsTransformerRear",
    .output_topic = "PointCloudFusion",
    .number_crunch_time = nodes::timing::Default::FUSION_NODE_CRUNCH_TIME}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Fusion>(
      nodes::FusionSettings{
    .node_name = "NDTLocalizer",
    .input_0 = "VoxelGridDownsampler",
    .input_1 = "PointCloudMapLoader",
    .output_topic = "NDTLocalizer",
    .number_crunch_time = nodes::timing::Default::FUSION_NODE_CRUNCH_TIME}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Fusion>(
      nodes::FusionSettings{
    .node_name = "NDTLocalizer",
    .input_0 = "VoxelGridDownsampler",
    .input_1 = "PointCloudMapLoader",
    .output_topic = "NDTLocalizer",
    .number_crunch_time = nodes::timing::Default::FUSION_NODE_CRUNCH_TIME}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Fusion>(
      nodes::FusionSettings{
    .node_name = "VehicleInterface",
    .input_0 = "MPCController",
    .input_1 = "BehaviorPlanner",
    .output_topic = "VehicleInterface",
    .number_crunch_time = nodes::timing::Default::FUSION_NODE_CRUNCH_TIME}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Fusion>(
      nodes::FusionSettings{
    .node_name = "Lanelet2GlobalPlanner",
    .input_0 = "rviz2",
    .input_1 = "NDTLocalizer",
    .output_topic = "Lanelet2GlobalPlanner",
    .number_crunch_time = nodes::timing::Default::FUSION_NODE_CRUNCH_TIME}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Fusion>(
      nodes::FusionSettings{
    .node_name = "Lanelet2MapLoader",
    .input_0 = "Lanelet2Map",
    .input_1 = "Lanelet2GlobalPlanner",
    .output_topic = "Lanelet2MapLoader",
    .number_crunch_time = nodes::timing::Default::FUSION_NODE_CRUNCH_TIME}));

  // reactor node
  nodes.emplace_back(
    std::make_shared<typename SystemType::Reactor>(
      nodes::ReactorSettings{.node_name = "BehaviorPlanner",
        .inputs = {"ObjectCollisionEstimator", "NDTLocalizer",
          "Lanelet2GlobalPlanner", "Lanelet2MapLoader",
          "ParkingPlanner", "LanePlanner"},
        .output_topic = "BehaviorPlanner",
        .number_crunch_time =
        nodes::timing::Default::REACTOR_NODE_CRUNCH_TIME}));

  // command node
  nodes.emplace_back(
    std::make_shared<typename SystemType::Command>(
      nodes::CommandSettings{
    .node_name = "VehicleDBWSystem", .input_topic = "VehicleInterface"}));
#pragma GCC diagnostic pop

  return nodes;
}

#endif  // REFERENCE_SYSTEM__SYSTEM_BUILDER_HPP_
