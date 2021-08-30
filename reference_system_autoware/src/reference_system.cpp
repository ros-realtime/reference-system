#include <chrono>
#include <vector>

#include "reference_system_autoware/node/sensor.hpp"

using namespace std::chrono_literals;

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  std::vector<std::shared_ptr<rclcpp::Node>> nodes;

  // setup communication graph
  nodes.emplace_back(std::make_shared<node::Sensor>(
      node::SensorSettings{.node_name = "FrontLidarDriver",
                           .topic_name = "FrontLidarDriver",
                           .cycle_time = 100ms}));

  nodes.emplace_back(std::make_shared<node::Sensor>(
      node::SensorSettings{.node_name = "RearLidarDriver",
                           .topic_name = "ReadLidarDriver",
                           .cycle_time = 100ms}));

  nodes.emplace_back(std::make_shared<node::Sensor>(
      node::SensorSettings{.node_name = "PointCloudMap",
                           .topic_name = "PointCloudMap",
                           .cycle_time = 100ms}));

  nodes.emplace_back(std::make_shared<node::Sensor>(node::SensorSettings{
      .node_name = "rviz2", .topic_name = "rviz2", .cycle_time = 100ms}));

  nodes.emplace_back(std::make_shared<node::Sensor>(
      node::SensorSettings{.node_name = "Lanelet2Map",
                           .topic_name = "Lanelet2Map",
                           .cycle_time = 100ms}));

  rclcpp::executors::MultiThreadedExecutor executor;
  for (auto& node : nodes) executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
