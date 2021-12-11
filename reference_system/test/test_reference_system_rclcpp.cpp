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
#include <gtest/gtest.h>

#include <string>

#include "test_fixtures.hpp"
#include "rclcpp/node_interfaces/node_graph.hpp"
#include "reference_system/system/systems.hpp"

// set the system to use
using SystemType = RclcppSystem;

TEST_F(TestNodeGraph, rclcpp_sensor_node) {
  auto settings = nodes::SensorSettings();
  settings.node_name = "SensorNode";
  settings.topic_name = settings.node_name;
  settings.cycle_time = milliseconds(100);
  // create node
  auto node =
    create_node<SystemType, SystemType::Sensor, nodes::SensorSettings>(settings);
  // confirm node was initialized with settings
  EXPECT_EQ(node->get_name(), settings.node_name);
  // get node graph of node
  auto * node_graph = node->get_node_graph_interface().get();
  ASSERT_NE(nullptr, node_graph);
  auto topic_names_and_types = node_graph->get_topic_names_and_types(false);
  // sensor nodes should publish one topic
  EXPECT_EQ(size_t(3), topic_names_and_types.size());
  EXPECT_EQ(size_t(1), node_graph->count_publishers(settings.topic_name));
}

TEST_F(TestNodeGraph, rclcpp_transform_node) {
  auto settings = nodes::TransformSettings();
  settings.node_name = "TransformNode";
  settings.input_topic = settings.node_name + "1";
  settings.output_topic = settings.node_name;
  settings.number_crunch_limit = CRUNCH;
  // create node
  auto node =
    create_node<SystemType, SystemType::Transform, nodes::TransformSettings>(settings);
  // confirm node was initialized with settings
  EXPECT_EQ(node->get_name(), settings.node_name);
  // get node graph of node
  auto * node_graph = node->get_node_graph_interface().get();
  ASSERT_NE(nullptr, node_graph);
  auto topic_names_and_types = node_graph->get_topic_names_and_types(false);
  // transform nodes should publish one topic and subscribe to one topic
  size_t pubs = 1;
  size_t subs = 1;
  size_t total_pubs_and_subs = pubs + subs + size_t(2);  // 2 for rosout and parameter_events
  EXPECT_EQ(total_pubs_and_subs, topic_names_and_types.size());
  EXPECT_EQ(pubs, node_graph->count_publishers(settings.output_topic));
  EXPECT_EQ(subs, node_graph->count_subscribers(settings.input_topic));
}

TEST_F(TestNodeGraph, rclcpp_intersection_node) {
  auto settings = nodes::IntersectionSettings();
  settings.node_name = "IntersectionNode";
  std::string input_topic = settings.node_name + "_in_";
  settings.connections.emplace_back(nodes::IntersectionSettings::Connection());
  settings.connections.emplace_back(nodes::IntersectionSettings::Connection());
  settings.connections[0].input_topic = input_topic + "1";
  settings.connections[0].output_topic = settings.node_name + "1";
  settings.connections[0].number_crunch_limit = CRUNCH;
  settings.connections[1].input_topic = input_topic + "2";
  settings.connections[1].output_topic = settings.node_name + "2";
  settings.connections[1].number_crunch_limit = CRUNCH;
  // create node
  auto node =
    create_node<SystemType, SystemType::Intersection, nodes::IntersectionSettings>(settings);
  // confirm node was initialized with settings
  EXPECT_EQ(node->get_name(), settings.node_name);
  // get node graph of node
  auto * node_graph = node->get_node_graph_interface().get();
  ASSERT_NE(nullptr, node_graph);
  auto topic_names_and_types = node_graph->get_topic_names_and_types(false);
  // intersection nodes should publish two topics and subscribe to two topics
  auto pubs = settings.connections.size();
  auto subs = settings.connections.size();
  auto total_pubs_and_subs = pubs + subs + 2;  // 2 for rosout and parameter_events
  EXPECT_EQ(total_pubs_and_subs, topic_names_and_types.size());
  for (auto connection : settings.connections) {
    EXPECT_EQ(size_t(1), node_graph->count_publishers(connection.output_topic));
    EXPECT_EQ(size_t(1), node_graph->count_subscribers(connection.input_topic));
  }
}

TEST_F(TestNodeGraph, rclcpp_fusion_node) {
  auto settings = nodes::FusionSettings();
  settings.node_name = "FusionNode";
  settings.input_0 = settings.node_name + "1";
  settings.input_1 = settings.node_name + "2";
  settings.number_crunch_limit = CRUNCH;
  settings.output_topic = settings.node_name;
  // create node
  auto node =
    create_node<SystemType, SystemType::Fusion, nodes::FusionSettings>(settings);
  // confirm node was initialized with settings
  EXPECT_EQ(node->get_name(), settings.node_name);
  // get node graph of node
  auto * node_graph = node->get_node_graph_interface().get();
  ASSERT_NE(nullptr, node_graph);
  auto topic_names_and_types = node_graph->get_topic_names_and_types(false);
  // intersection nodes should publish two topics and subscribe to two topics
  size_t pubs = 1;
  size_t subs = 2;
  size_t total_pubs_and_subs = pubs + subs + 2;  // 2 for rosout and parameter_events
  EXPECT_EQ(total_pubs_and_subs, topic_names_and_types.size());
  EXPECT_EQ(size_t(1), node_graph->count_publishers(settings.node_name));
  EXPECT_EQ(size_t(1), node_graph->count_subscribers(settings.input_0));
  EXPECT_EQ(size_t(1), node_graph->count_subscribers(settings.input_1));
}

TEST_F(TestNodeGraph, rclcpp_cyclic_node) {
  auto settings = nodes::CyclicSettings();
  settings.node_name = "CyclicNode";
  settings.inputs.emplace_back(settings.node_name + "1");
  settings.inputs.emplace_back(settings.node_name + "2");
  settings.inputs.emplace_back(settings.node_name + "3");
  settings.number_crunch_limit = CRUNCH;
  settings.output_topic = settings.node_name;
  // create node
  auto node =
    create_node<SystemType, SystemType::Cyclic, nodes::CyclicSettings>(settings);
  // confirm node was initialized with settings
  EXPECT_EQ(node->get_name(), settings.node_name);
  // get node graph of node
  auto * node_graph = node->get_node_graph_interface().get();
  ASSERT_NE(nullptr, node_graph);
  auto topic_names_and_types = node_graph->get_topic_names_and_types(false);
  // intersection nodes should publish two topics and subscribe to two topics
  size_t pubs = 1;
  size_t subs = 3;
  auto total_pubs_and_subs = pubs + subs + 2;  // 2 for rosout and parameter_events
  EXPECT_EQ(total_pubs_and_subs, topic_names_and_types.size());
  EXPECT_EQ(size_t(1), node_graph->count_publishers(settings.node_name));
  EXPECT_EQ(size_t(1), node_graph->count_subscribers(settings.inputs[0]));
  EXPECT_EQ(size_t(1), node_graph->count_subscribers(settings.inputs[1]));
  EXPECT_EQ(size_t(1), node_graph->count_subscribers(settings.inputs[2]));
}

TEST_F(TestNodeGraph, rclcpp_command_node) {
  auto settings = nodes::CommandSettings();
  settings.node_name = "CommandNode";
  settings.input_topic = settings.node_name + "_in";
  // create node
  auto node =
    create_node<SystemType, SystemType::Command, nodes::CommandSettings>(settings);
  // confirm node was initialized with settings
  EXPECT_EQ(node->get_name(), settings.node_name);
  // get node graph of node
  auto * node_graph = node->get_node_graph_interface().get();
  ASSERT_NE(nullptr, node_graph);
  auto topic_names_and_types = node_graph->get_topic_names_and_types(false);
  // intersection nodes should publish two topics and subscribe to two topics
  size_t pubs = 0;
  size_t subs = 1;
  auto total_pubs_and_subs = pubs + subs + 2;  // 2 for rosout and parameter_events
  EXPECT_EQ(total_pubs_and_subs, topic_names_and_types.size());
  EXPECT_EQ(size_t(0), node_graph->count_publishers(settings.node_name));
  EXPECT_EQ(size_t(1), node_graph->count_subscribers(settings.input_topic));
}
