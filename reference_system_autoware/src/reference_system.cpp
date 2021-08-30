#include <chrono>
#include <vector>

#include "reference_system_autoware/node/fusion.hpp"
#include "reference_system_autoware/node/processing.hpp"
#include "reference_system_autoware/node/sensor.hpp"

using namespace std::chrono_literals;

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  std::vector<std::shared_ptr<rclcpp::Node>> nodes;

  // setup communication graph
  // sensor nodes
  constexpr auto CYCLE_TIME = 100ms;
  nodes.emplace_back(std::make_shared<node::Sensor>(
      node::SensorSettings{.node_name = "FrontLidarDriver",
                           .topic_name = "FrontLidarDriver",
                           .cycle_time = CYCLE_TIME}));

  nodes.emplace_back(std::make_shared<node::Sensor>(
      node::SensorSettings{.node_name = "RearLidarDriver",
                           .topic_name = "RearLidarDriver",
                           .cycle_time = CYCLE_TIME}));

  nodes.emplace_back(std::make_shared<node::Sensor>(
      node::SensorSettings{.node_name = "PointCloudMap",
                           .topic_name = "PointCloudMap",
                           .cycle_time = CYCLE_TIME}));

  nodes.emplace_back(std::make_shared<node::Sensor>(node::SensorSettings{
      .node_name = "rviz2", .topic_name = "rviz2", .cycle_time = CYCLE_TIME}));

  nodes.emplace_back(std::make_shared<node::Sensor>(
      node::SensorSettings{.node_name = "Lanelet2Map",
                           .topic_name = "Lanelet2Map",
                           .cycle_time = CYCLE_TIME}));

  // processing nodes
  constexpr auto PROCESSING_TIME = 100ms;
  nodes.emplace_back(std::make_shared<node::Processing>(
      node::ProcessingSettings{.node_name = "PointsTransformerFront",
                               .input_topic = "FrontLidarDriver",
                               .output_topic = "PointsTransformerFront",
                               .number_crunch_time = PROCESSING_TIME}));

  nodes.emplace_back(std::make_shared<node::Processing>(
      node::ProcessingSettings{.node_name = "PointsTransformerRear",
                               .input_topic = "RearLidarDriver",
                               .output_topic = "PointsTransformerRear",
                               .number_crunch_time = PROCESSING_TIME}));

  nodes.emplace_back(std::make_shared<node::Processing>(
      node::ProcessingSettings{.node_name = "VoxelGridDownsampler",
                               .input_topic = "PointCloudFusion",
                               .output_topic = "VoxelGridDownsampler",
                               .number_crunch_time = PROCESSING_TIME}));

  nodes.emplace_back(std::make_shared<node::Processing>(
      node::ProcessingSettings{.node_name = "PointCloudMapLoader",
                               .input_topic = "PointCloudMap",
                               .output_topic = "PointCloudMapLoader",
                               .number_crunch_time = PROCESSING_TIME}));

  nodes.emplace_back(std::make_shared<node::Processing>(
      node::ProcessingSettings{.node_name = "RayGroundFilter",
                               .input_topic = "PointCloudFusion",
                               .output_topic = "RayGroundFilter",
                               .number_crunch_time = PROCESSING_TIME}));

  nodes.emplace_back(std::make_shared<node::Processing>(
      node::ProcessingSettings{.node_name = "EuclideanClusterDetector",
                               .input_topic = "RayGroundFilter",
                               .output_topic = "EuclideanClusterDetector",
                               .number_crunch_time = PROCESSING_TIME}));

  nodes.emplace_back(std::make_shared<node::Processing>(
      node::ProcessingSettings{.node_name = "ObjectCollisionEstimator",
                               .input_topic = "EuclideanClusterDetector",
                               .output_topic = "ObjectCollisionEstimator",
                               .number_crunch_time = PROCESSING_TIME}));

  nodes.emplace_back(std::make_shared<node::Processing>(
      node::ProcessingSettings{.node_name = "MPCController",
                               .input_topic = "BehaviorPlanner",
                               .output_topic = "MPCController",
                               .number_crunch_time = PROCESSING_TIME}));

  nodes.emplace_back(std::make_shared<node::Processing>(
      node::ProcessingSettings{.node_name = "ParkingPlanner",
                               .input_topic = "Lanelet2MapLoader",
                               .output_topic = "ParkingPlanner",
                               .number_crunch_time = PROCESSING_TIME}));

  nodes.emplace_back(std::make_shared<node::Processing>(
      node::ProcessingSettings{.node_name = "LanePlanner",
                               .input_topic = "Lanelet2MapLoader",
                               .output_topic = "LanePlanner",
                               .number_crunch_time = PROCESSING_TIME}));

  // fusion nodes
  constexpr auto FUSION_TIME = 100ms;
  nodes.emplace_back(std::make_shared<node::Fusion>(
      node::FusionSettings{.node_name = "PointCloudFusion",
                           .input_0 = "PointsTransformerFront",
                           .input_1 = "PointsTransformerRear",
                           .output_topic = "PointCloudFusion",
                           .number_crunch_time = FUSION_TIME}));

  nodes.emplace_back(std::make_shared<node::Fusion>(
      node::FusionSettings{.node_name = "NDTLocalizer",
                           .input_0 = "VoxelGridDownsampler",
                           .input_1 = "PointCloudMapLoader",
                           .output_topic = "NDTLocalizer",
                           .number_crunch_time = FUSION_TIME}));

  nodes.emplace_back(std::make_shared<node::Fusion>(
      node::FusionSettings{.node_name = "NDTLocalizer",
                           .input_0 = "VoxelGridDownsampler",
                           .input_1 = "PointCloudMapLoader",
                           .output_topic = "NDTLocalizer",
                           .number_crunch_time = FUSION_TIME}));

  nodes.emplace_back(std::make_shared<node::Fusion>(
      node::FusionSettings{.node_name = "VehicleInterface",
                           .input_0 = "MPCController",
                           .input_1 = "BehaviorPlanner",
                           .output_topic = "VehicleInterface",
                           .number_crunch_time = FUSION_TIME}));

  nodes.emplace_back(std::make_shared<node::Fusion>(
      node::FusionSettings{.node_name = "Lanelet2GlobalPlanner",
                           .input_0 = "rviz2",
                           .input_1 = "NDTLocalizer",
                           .output_topic = "Lanelet2GlobalPlanner",
                           .number_crunch_time = FUSION_TIME}));

  nodes.emplace_back(std::make_shared<node::Fusion>(
      node::FusionSettings{.node_name = "Lanelet2MapLoader",
                           .input_0 = "Lanelet2Map",
                           .input_1 = "Lanelet2GlobalPlanner",
                           .output_topic = "Lanelet2MapLoader",
                           .number_crunch_time = PROCESSING_TIME}));

  rclcpp::executors::MultiThreadedExecutor executor;
  for (auto& node : nodes) executor.add_node(node);
  executor.spin();

  nodes.clear();
  rclcpp::shutdown();
  return 0;
}
