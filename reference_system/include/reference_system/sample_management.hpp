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
#ifndef REFERENCE_SYSTEM__SAMPLE_MANAGEMENT_HPP_
#define REFERENCE_SYSTEM__SAMPLE_MANAGEMENT_HPP_
#include <algorithm>
#include <chrono>
#include <map>
#include <string>
#include <iostream>

#include "reference_system/msg_types.hpp"

bool set_benchmark_mode(const bool has_benchmark_mode, const bool set_value = true)
{
  static bool value{false};
  if (set_value) {value = has_benchmark_mode;}
  return value;
}

bool is_in_benchmark_mode()
{
  return set_benchmark_mode(false, false);
}

template<typename SampleTypePointer>
void set_sample(const std::string & node_name, SampleTypePointer & sample)
{
  if (is_in_benchmark_mode() ) {return;}

  if (sample.size >= message_t::STATS_CAPACITY) {
    return;
  }

  uint64_t idx = sample.size;
  ++sample.size;

  memcpy(
    sample.stats[idx].node_name.data(), node_name.data(),
    std::min(
      node_name.size(),
      reference_interfaces::msg::TransmissionStats::NODE_NAME_LENGTH));

  sample.stats[idx].timestamp = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch())
    .count());
}

template<typename SampleTypePointer>
uint64_t get_sample_timestamp(SampleTypePointer & sample)
{
  if (is_in_benchmark_mode() || sample->size == 0) {
    return 0;
  } else {
    return sample->stats[sample->size - 1].timestamp;
  }
}

template<typename SampleTypePointer, typename SourceType>
void fuse_samples(
  const std::string & node_name, SampleTypePointer & destination,
  const SourceType & source)
{
  if (is_in_benchmark_mode() ) {return;}

  destination.size = source->size;
  destination.stats = source->stats;

  set_sample(node_name, destination);
}

template<typename SampleTypePointer, typename SourceType>
void fuse_samples(
  const std::string & node_name, SampleTypePointer & destination,
  const SourceType & source1, const SourceType & source2)
{
  if (is_in_benchmark_mode() ) {return;}

  uint64_t elements_to_copy =
    std::min(message_t::STATS_CAPACITY, source1->size + source2->size);

  destination.size = elements_to_copy;

  destination.stats = source1->stats;
  memcpy(
    destination.stats.data() + source1->size, source2->stats.data(),
    sizeof(reference_interfaces::msg::TransmissionStats) *
    (elements_to_copy - source1->size));

  set_sample(node_name, destination);
}

template<typename SampleTypePointer>
void print_sample_path(
  const std::string & node_name,
  const SampleTypePointer & sample)
{
  if (is_in_benchmark_mode() || sample->size <= 0) {return;}

  struct sample_statistic_t
  {
    uint64_t number_of_received_samples = 0;
    uint64_t number_of_hot_path_samples = 0;
    uint64_t timepoint_of_first_received_sample = 0;

    struct statistic_value_t
    {
      uint64_t average = 0;
      uint64_t min = std::numeric_limits<uint64_t>::max();
      uint64_t max = 0;

      void set(const uint64_t value, const uint64_t total_number)
      {
        average = ((total_number - 1) * average + value) / total_number;
        min = std::min(min, value);
        max = std::max(max, value);
      }
    };

    statistic_value_t latency;
    statistic_value_t hot_path_latency;
  };

  static std::map<std::string, sample_statistic_t> advanced_statistics;
  auto iter = advanced_statistics.find(node_name);
  if (iter == advanced_statistics.end() ) {
    advanced_statistics[node_name].timepoint_of_first_received_sample =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  }

  advanced_statistics[node_name].number_of_received_samples++;

  const uint64_t timestamp_in_ns = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch())
    .count());

  std::cout << "----------------------------------------------------------" <<
    std::endl;
  std::cout << "sample path: " << std::endl;
  std::cout << "  timepoint             node name" << std::endl;

  std::map<uint64_t, uint64_t> timestamp2Order;
  uint64_t min_time_stamp = std::numeric_limits<uint64_t>::max();
  uint64_t max_time_stamp = 0;

  for (uint64_t i = 0; i < sample->size; ++i) {
    timestamp2Order[sample->stats[i].timestamp] = 0;
    min_time_stamp = std::min(min_time_stamp, sample->stats[i].timestamp);
    max_time_stamp = std::max(max_time_stamp, sample->stats[i].timestamp);
  }
  uint64_t i = 0;
  for (auto & e : timestamp2Order) {
    e.second = i++;
  }

  for (uint64_t i = 0; i < sample->size; ++i) {
    std::cout << "  [";
    std::cout.width(2);
    std::cout << timestamp2Order[sample->stats[i].timestamp];
    std::cout << "] " << sample->stats[i].timestamp << " : " <<
      sample->stats[i].node_name.data() << std::endl;
  }

  uint64_t hot_path_latency_in_ns = 0;
  bool does_contain_hot_path = false;
  for (uint64_t i = 0; i < sample->size; ++i) {
    uint64_t idx = sample->size - i - 1;
    std::string current_node_name(
      reinterpret_cast<const char *>(sample->stats[idx].node_name.data()));

    if (current_node_name == "EuclideanClusterDetector") {
      hot_path_latency_in_ns = sample->stats[idx].timestamp;
    } else if (current_node_name == "FrontLidarDriver" &&
      hot_path_latency_in_ns != 0)
    {
      hot_path_latency_in_ns = hot_path_latency_in_ns - sample->stats[idx].timestamp;
      does_contain_hot_path = true;
      break;
    }
  }

  uint64_t latency_in_ns = max_time_stamp - min_time_stamp;
  advanced_statistics[node_name].latency.set(latency_in_ns,
    advanced_statistics[node_name].number_of_received_samples);
  uint64_t latency_min_in_ns = advanced_statistics[node_name].latency.min;
  uint64_t latency_max_in_ns = advanced_statistics[node_name].latency.max;
  uint64_t latency_average_in_ns = advanced_statistics[node_name].latency.average;

  std::cout << std::endl;
  std::cout << "Statistics:" << std::endl;
  std::cout << "  destination:      " << node_name << std::endl;
  std::cout << "  current time:     " << timestamp_in_ns << std::endl;
  std::cout << "  latency:          " << static_cast<double>(latency_in_ns) / 1000000.0 << " ms" <<
    "  [ min = " << static_cast<double>(latency_min_in_ns) / 1000000.0 << " ms, max = " <<
    static_cast<double>(latency_max_in_ns) / 1000000.0 << " ms, average = " <<
    static_cast<double>(latency_average_in_ns) / 1000000.0 << " ms ]" << std::endl;

  if (does_contain_hot_path) {
    advanced_statistics[node_name].number_of_hot_path_samples++;
    advanced_statistics[node_name].hot_path_latency.set(hot_path_latency_in_ns,
      advanced_statistics[node_name].number_of_hot_path_samples);
    uint64_t hot_path_min_in_ns = advanced_statistics[node_name].hot_path_latency.min;
    uint64_t hot_path_max_in_ns = advanced_statistics[node_name].hot_path_latency.max;
    uint64_t hot_path_average_in_ns = advanced_statistics[node_name].hot_path_latency.average;
    std::cout << "  hot path:         FrontLidarDriver -> EuclideanClusterDetector" << std::endl;
    std::cout << "  hot path latency: " <<
      static_cast<double>(hot_path_latency_in_ns) / 1000000.0 << " ms" <<
      "  [ min = " << static_cast<double>(hot_path_min_in_ns) / 1000000.0 << " ms, max = " <<
      static_cast<double>(hot_path_max_in_ns) / 1000000.0 << " ms, average = " <<
      static_cast<double>(hot_path_average_in_ns) / 1000000.0 << " ms ]" << std::endl;
  }

  std::cout << "----------------------------------------------------------" <<
    std::endl;
  std::cout << std::endl;
}

#endif  // REFERENCE_SYSTEM__SAMPLE_MANAGEMENT_HPP_
