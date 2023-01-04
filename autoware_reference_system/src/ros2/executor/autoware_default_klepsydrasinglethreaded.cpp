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

#include "rclcpp/rclcpp.hpp"
#include <klepsydra/mem_core/mem_env.h>
#include "reference_system/system/systems.hpp"

#include "autoware_reference_system/autoware_system_builder.hpp"
#include "autoware_reference_system/system/timing/benchmark.hpp"
#include "autoware_reference_system/system/timing/default.hpp"

#include <kpsr_ros2_executor/executor_factory.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  using TimeConfig = nodes::timing::Default;
  // uncomment for benchmarking
  // using TimeConfig = nodes::timing::BenchmarkCPUUsage;
  // set_benchmark_mode(true);

  auto nodes = create_autoware_nodes<RclcppSystem, TimeConfig>();

  kpsr::mem::MemEnv environment;
  /*
  environment.setPropertyString("container_name", "eventLoop_systemRef");
  environment.setPropertyString("admin_log_filename", "");
  environment.setPropertyString("stat_filename", "/home/ubuntu/development/ros2/ref_results/ros_ref.csv");
  environment.setPropertyString("streaming_conf_file", "/home/ubuntu/development/ros2/ref_results/outputPolicy.json");
  environment.setPropertyString("log_filename", "/home/ubuntu/development/ros2/ref_results/ros_ref.log");

  environment.setPropertyInt("log_level", 2);
  environment.setPropertyInt("pool_size", 100);
  environment.setPropertyInt("number_of_cores", 3);
  environment.setPropertyInt("critical_thread_pool_size", 16);
  environment.setPropertyInt("non_critical_thread_pool_size", 1);
  environment.setPropertyInt("number_of_parallel_threads", 0);
  environment.setPropertyInt("stat_log_interval_ms", 1000);
  environment.setPropertyInt("admin_log_interval", 1000);

  environment.setPropertyBool("to_std_out", true);
  environment.setPropertyBool("log_to_file", true);
  environment.setPropertyBool("admin_log_to_file", false);
  environment.setPropertyBool("stat_socket_container_enable", false);
  environment.setPropertyBool("stat_file_container_enable", true);
  environment.setPropertyBool("use_default_streaming_factory", true);
  environment.setPropertyBool("test_dnn", false);
  environment.setPropertyBool("export_streaming_configuration", true);
  */
  environment.setPropertyString("log_filename", "");
  environment.setPropertyInt("log_level", 2);
  environment.setPropertyInt("pool_size", 1);
  environment.setPropertyInt("number_of_cores", 4);//1,2,3,4
  environment.setPropertyInt("number_of_trypost", 0);
  environment.setPropertyInt("critical_thread_pool_size", 32);
  environment.setPropertyInt("non_critical_thread_pool_size", 32);
  environment.setPropertyInt("number_of_parallel_threads", 0);

  //
  environment.setPropertyInt("stat_log_interval_ms", 1000);
  environment.setPropertyBool("stat_file_container_enable", false);
  environment.setPropertyString("stat_filename", "/home/ubuntu/development/ros2/ros2_core_ws/stats/benchmarks_internal_sub.csv");
  //
  environment.setPropertyString("streaming_conf_file", "/home/ubuntu/development/ros2/ros2_core_ws/stats/policy.json");
  environment.setPropertyBool("use_default_streaming_factory", false);
  environment.setPropertyBool("test_dnn", false);
  environment.setPropertyBool("export_streaming_configuration", false);

  rclcpp::Executor::SharedPtr executor = kpsr::ros2::ExecutorFactory::createExecutor(&environment);

  for (auto & node : nodes) {
    executor->add_node(node);
  }
  executor->spin();

  nodes.clear();
  rclcpp::shutdown();

  return 0;
}
