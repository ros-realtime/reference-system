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
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <cmath>
#include <set>

#include "reference_system/msg_types.hpp"

/// A singleton for sample management settings that may differ between experiments
class SampleManagementSettings
{
public:
  static SampleManagementSettings & get()
  {
    static SampleManagementSettings settings;
    return settings;
  }

  void set_hot_path(
    const std::set<std::string> & hot_path_nodes,
    const std::set<std::string> & hot_path_roots,
    const std::string & hot_path_sink)
  {
    m_hot_path_nodes = hot_path_nodes;
    m_hot_path_roots = hot_path_roots;
    m_hot_path_sink = hot_path_sink;
  }
  const std::string & hot_path_sink() const noexcept
  {
    return m_hot_path_sink;
  }
  bool is_hot_path_root(const std::string & name) const noexcept
  {
    return m_hot_path_roots.count(name) > 0;
  }
  bool is_hot_path_node(const std::string & name) const noexcept
  {
    return m_hot_path_nodes.count(name) > 0;
  }
  std::string hot_path_name() const noexcept
  {
    std::string roots_name;
    for (const auto & root : m_hot_path_roots) {
      if (roots_name != "") {
        roots_name += "/";
      }
      roots_name += root;
    }

    return roots_name + " -> " + hot_path_sink();
  }

  void set_benchmark_mode(bool value)
  {
    m_benchmark_mode = value;
  }
  bool is_in_benchmark_mode() const noexcept
  {
    return m_benchmark_mode;
  }

private:
  bool m_benchmark_mode;
  std::set<std::string> m_hot_path_nodes;
  std::set<std::string> m_hot_path_roots;
  std::string m_hot_path_sink;

  SampleManagementSettings()
  : m_benchmark_mode(false) {}
};
void set_benchmark_mode(const bool benchmark_mode)
{
  SampleManagementSettings::get().set_benchmark_mode(benchmark_mode);
}
bool is_in_benchmark_mode()
{
  return SampleManagementSettings::get().is_in_benchmark_mode();
}

uint64_t now_as_int()
{
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

template<typename SampleTypePointer>
void set_sample(
  const std::string & node_name, const uint32_t sequence_number,
  const uint32_t dropped_samples, const uint64_t timestamp,
  SampleTypePointer & sample)
{
  if (is_in_benchmark_mode()) {
    return;
  }

  if (sample.size >= message_t::STATS_CAPACITY) {
    return;
  }

  uint64_t idx = sample.size;
  ++sample.size;

  const size_t name_len = std::min(
    node_name.size(),
    sample.stats[idx].node_name.size() - 1);
  memcpy(sample.stats[idx].node_name.data(), node_name.data(), name_len);
  sample.stats[idx].node_name[name_len] = '\0';

  sample.stats[idx].timestamp = timestamp;

  sample.stats[idx].sequence_number = sequence_number;
  sample.stats[idx].dropped_samples = dropped_samples;
}

template<typename SampleTypePointer>
uint64_t get_sample_timestamp(const SampleTypePointer & sample)
{
  if (is_in_benchmark_mode() || sample->size == 0) {
    return 0;
  } else {
    return sample->stats[sample->size - 1].timestamp;
  }
}

template<typename SampleTypePointer>
uint32_t get_sample_sequence_number(const SampleTypePointer & sample)
{
  if (is_in_benchmark_mode() || sample->size == 0) {
    return 0;
  } else {
    return sample->stats[sample->size - 1].sequence_number;
  }
}

template<typename SampleTypePointer>
uint32_t get_missed_samples_and_update_seq_nr(
  const SampleTypePointer & sample,
  uint32_t & sequence_number)
{
  uint32_t updated_seq_nr = get_sample_sequence_number(sample);
  uint32_t missed_samples = (updated_seq_nr > sequence_number) ?
    updated_seq_nr - sequence_number - 1 :
    0;
  sequence_number = updated_seq_nr;
  return missed_samples;
}

template<typename SampleTypePointer, typename SourceType>
void merge_history_into_sample(
  SampleTypePointer & sample,
  const SourceType & source)
{
  if (is_in_benchmark_mode()) {
    return;
  }

  std::vector<uint64_t> entries_to_add;

  for (uint64_t i = 0; i < source->size; ++i) {
    bool entry_found = false;
    // The unsigned char* returned by data() needs to be casted to char*
    std::string source_name(reinterpret_cast<const char *>(source->stats[i].node_name.data()));

    for (uint64_t k = 0; k < sample.size; ++k) {
      // The unsigned char* returned by data() needs to be casted to char*
      std::string sample_name(reinterpret_cast<const char *>(sample.stats[k].node_name.data()));
      if (source_name == sample_name) {
        entry_found = true;
        break;
      }
    }

    if (!entry_found) {
      entries_to_add.emplace_back(i);
    }
  }

  for (auto i : entries_to_add) {
    memcpy(
      sample.stats.data() + sample.size, source->stats.data() + i,
      sizeof(reference_interfaces::msg::TransmissionStats));
    ++sample.size;
  }
}

struct statistic_value_t
{
  double average = 0.0;
  double deviation = 0.0;
  uint64_t min = std::numeric_limits<uint64_t>::max();
  uint64_t max = 0;
  uint64_t current = 0;
  double total_number = 0.0;
  std::string suffix;
  double adjustment = 0.0;
  double m2 = 0.0;

  void set(const uint64_t value)
  {
    // Use Welford's online algorithm to calculate deviation
    ++total_number;
    current = value;
    // Ensure uint64_t is smaller than double on the current system
    if (std::numeric_limits<uint64_t>::max() > std::numeric_limits<double>::max()) {
      if (static_cast<double>(value) > std::numeric_limits<double>::max()) {
        throw std::overflow_error("Passed statistics sample is too large for a double");
      }
    }
    double value_d = static_cast<double>(value);
    auto previous_delta = value_d - average;
    average += previous_delta / total_number;
    auto new_delta = value_d - average;
    m2 += (previous_delta * new_delta);
    deviation = std::sqrt(m2 / (total_number - 1));
    min = std::min(min, value);
    max = std::max(max, value);
  }
};

struct sample_statistic_t
{
  uint64_t timepoint_of_first_received_sample = 0;
  uint32_t previous_behavior_planner_sequence = 0;
  uint64_t previous_behavior_planner_time_stamp = 0;

  statistic_value_t latency;
  statistic_value_t hot_path_latency;
  statistic_value_t behavior_planner_period;
};

std::ostream & operator<<(std::ostream & output, const statistic_value_t & v)
{
  if (v.adjustment == 0.0) {
    output << v.current << v.suffix << " [min=" << v.min << v.suffix <<
      ", max=" << v.max << v.suffix << ", average=" << v.average <<
      v.suffix << ", deviation=" << v.deviation << "]";
  } else {
    output << static_cast<double>(v.current) / v.adjustment << v.suffix <<
      " [min=" << static_cast<double>(v.min) / v.adjustment << v.suffix <<
      ", max=" << static_cast<double>(v.max) / v.adjustment << v.suffix <<
      ", average=" << v.average / v.adjustment << v.suffix <<
      ", deviation=" << v.deviation / v.adjustment << v.suffix << "]";
  }
  return output;
}

template<typename SampleTypePointer>
void print_sample_path(
  const std::string & node_name,
  const uint32_t lost_samples,
  const SampleTypePointer & sample)
{
  static int benchmark_counter = 0;
  ++benchmark_counter;

  // benchmark_counter = dismissing first 100 samples to get rid of startup
  // jitter
  if (is_in_benchmark_mode() || sample->size <= 0 || benchmark_counter < 10) {
    return;
  }

  static std::map<std::string, sample_statistic_t> advanced_statistics;
  static std::map<std::string, std::map<std::string, statistic_value_t>>
  dropped_samples;

  auto iter = advanced_statistics.find(node_name);
  if (iter == advanced_statistics.end()) {
    advanced_statistics[node_name].timepoint_of_first_received_sample = now_as_int();
    advanced_statistics[node_name].latency.suffix = "ms";
    advanced_statistics[node_name].latency.adjustment = 1000000.0;
    advanced_statistics[node_name].hot_path_latency.suffix = "ms";
    advanced_statistics[node_name].hot_path_latency.adjustment = 1000000.0;
    advanced_statistics[node_name].behavior_planner_period.suffix = "ms";
    advanced_statistics[node_name].behavior_planner_period.adjustment =
      1000000.0;
  }

  const uint64_t timestamp_in_ns = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch())
    .count());

  static std::mutex cout_mutex;
  std::lock_guard<std::mutex> lock(cout_mutex);

  std::cout << "----------------------------------------------------------" <<
    std::endl;
  std::cout << "sample path: (" << node_name << ") " << std::endl;
  std::cout << "  order timepoint           sequence nr.                  node "
    "name     dropped samples" <<
    std::endl;

  std::map<uint64_t, uint64_t> timestamp2Order;
  uint64_t min_time_stamp = std::numeric_limits<uint64_t>::max();

  for (uint64_t i = 0; i < sample->size; ++i) {
    timestamp2Order[sample->stats[i].timestamp] = 0;
    min_time_stamp = std::min(min_time_stamp, sample->stats[i].timestamp);
  }
  uint64_t i = 0;
  for (auto & e : timestamp2Order) {
    e.second = i++;
  }

  (void)lost_samples;  // to avoid unused param warning in Clang

#if 0
  for (uint64_t i = 0; i < sample->size; ++i) {
    std::string name((const char *)sample->stats[i].node_name.data());
    std::cout << "  [";
    std::cout.width(2);
    std::cout << timestamp2Order[sample->stats[i].timestamp];
    std::cout << "]  " << sample->stats[i].timestamp << "  ";
    std::cout.width(10);
    std::cout << sample->stats[i].sequence_number;
    std::cout << "    ";
    std::cout.width(24);
    std::cout << name;
    std::cout << "     ";
    dropped_samples[node_name][name].set(sample->stats[i].dropped_samples);
    std::cout << dropped_samples[node_name][name];
    std::cout << std::endl;
  }

  std::cout << "  [";
  std::cout.width(2);
  std::cout << timestamp2Order.size();
  std::cout << "]  " << timestamp_in_ns << "  ";
  std::cout.width(10);
  std::cout << "endpoint";
  std::cout << "    ";
  std::cout.width(24);
  std::cout << node_name;
  std::cout << "     ";
  dropped_samples[node_name][node_name].set(lost_samples);
  std::cout << dropped_samples[node_name][node_name];
  std::cout << std::endl;
#endif
  // hot path latency
  uint64_t hot_path_latency_in_ns = 0;
  bool does_contain_hot_path = false;
  uint64_t root_timestamp = 0;
  const auto settings = SampleManagementSettings::get();
  for (uint64_t i = 0; i < sample->size; ++i) {
    uint64_t idx = sample->size - i - 1;
    std::string current_node_name(
      reinterpret_cast<const char *>(sample->stats[idx].node_name.data()));

    if (settings.is_hot_path_root(current_node_name)) {
      root_timestamp = std::max(root_timestamp, sample->stats[idx].timestamp);
    }
    if (current_node_name == settings.hot_path_sink()) {
      hot_path_latency_in_ns = sample->stats[idx].timestamp;
      does_contain_hot_path = true;
    }
  }
  hot_path_latency_in_ns -= root_timestamp;

  // hot path drops
  uint64_t hot_path_drops = 0;
  if (does_contain_hot_path) {
    for (uint64_t i = 0; i < sample->size; ++i) {
      uint64_t idx = sample->size - i - 1;
      std::string current_node_name(
        reinterpret_cast<const char *>(sample->stats[idx].node_name.data()));
      if (settings.is_hot_path_node(current_node_name) > 0) {
        hot_path_drops += sample->stats[idx].dropped_samples;
      }
    }
  }

  // behavior planner cycle time
  bool does_contain_behavior_planner = false;
  for (uint64_t i = 0; i < sample->size; ++i) {
    std::string current_node_name(
      reinterpret_cast<const char *>(sample->stats[i].node_name.data()));
    if (current_node_name == "BehaviorPlanner") {
      does_contain_behavior_planner = true;
      auto seq_nr = sample->stats[i].sequence_number;
      auto timestamp = sample->stats[i].timestamp;
      auto prev_seq_nr =
        advanced_statistics[node_name].previous_behavior_planner_sequence;
      auto prev_timestamp =
        advanced_statistics[node_name].previous_behavior_planner_time_stamp;
      if (prev_timestamp != 0) {
        std::cout << "bp period" << std::endl;
        advanced_statistics[node_name].behavior_planner_period.set(
          static_cast<uint64_t>(static_cast<double>(timestamp - prev_timestamp) /
          static_cast<double>(seq_nr - prev_seq_nr)));
      }
      advanced_statistics[node_name].previous_behavior_planner_sequence =
        seq_nr;
      advanced_statistics[node_name].previous_behavior_planner_time_stamp =
        timestamp;
    }
  }

  std::cout << "latency" << std::endl;
  advanced_statistics[node_name].latency.set(timestamp_in_ns - min_time_stamp);

  std::cout << std::endl;
  std::cout << "Statistics:" << std::endl;
  std::cout << "  latency:                  " <<
    advanced_statistics[node_name].latency << std::endl;

  if (does_contain_hot_path) {
    std::cout << "hotpath" << std::endl;
    dropped_samples[node_name]["hotpath"].set(hot_path_drops);
    advanced_statistics[node_name].hot_path_latency.set(hot_path_latency_in_ns);
    std::cout << "  hot path:                 " << settings.hot_path_name() << std::endl;
    std::cout << "  hot path latency:         " <<
      advanced_statistics[node_name].hot_path_latency << std::endl;
    std::cout << "  hot path drops:           " <<
      dropped_samples[node_name]["hotpath"] << std::endl;
  }

  if (does_contain_behavior_planner) {
    std::cout << "  behavior planner period:  " <<
      advanced_statistics[node_name].behavior_planner_period <<
      std::endl;
  }

  std::cout << "----------------------------------------------------------" <<
    std::endl;
  std::cout << std::endl;
}

#endif  // REFERENCE_SYSTEM__SAMPLE_MANAGEMENT_HPP_
