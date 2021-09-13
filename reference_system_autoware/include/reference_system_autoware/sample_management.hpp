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
#ifndef REFERENCE_SYSTEM_AUTOWARE__SAMPLE_MANAGEMENT_HPP_
#define REFERENCE_SYSTEM_AUTOWARE__SAMPLE_MANAGEMENT_HPP_
#include <algorithm>
#include <chrono>
#include <map>
#include <string>

#include "reference_system_autoware/types.hpp"


template<typename SampleType>
void set_sample(const std::string & node_name, SampleType & sample)
{
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

  sample.stats[idx].timestamp =
    std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch())
    .count();
}

template<typename SampleType, typename SourceType>
void fuse_samples(
  const std::string & node_name, SampleType & destination,
  const SourceType & source)
{
  destination.size = source->size;
  destination.stats = source->stats;

  set_sample(node_name, destination);
}

template<typename SampleType, typename SourceType>
void fuse_samples(
  const std::string & node_name, SampleType & destination,
  const SourceType & source1, const SourceType & source2)
{
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

template<typename SampleType>
void print_sample_path(const std::string & node_name, const SampleType & sample)
{
  const int64_t timestamp_in_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch())
    .count();

  std::cout << "----------------------------------------------------------" <<
    std::endl;
  std::cout << "sample path: " << std::endl;
  std::cout << "  timepoint             node name" << std::endl;

  std::map<uint64_t, uint64_t> timestamp2Order;
  for (uint64_t i = 0; i < sample->size; ++i) {
    timestamp2Order[sample->stats[i].timestamp] = 0;
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

  uint64_t latency_in_ns = 0;
  if (sample->size > 0) {
    latency_in_ns = timestamp_in_ns - sample->stats[sample->size - 1].timestamp;
  }

  std::cout << std::endl;
  std::cout << "  destination:  " << node_name << std::endl;
  std::cout << "  current time: " << timestamp_in_ns << std::endl;
  std::cout << "  latency:      " << latency_in_ns << std::endl;
  std::cout << "----------------------------------------------------------" <<
    std::endl;
  std::cout << std::endl;
}

#endif  // REFERENCE_SYSTEM_AUTOWARE__SAMPLE_MANAGEMENT_HPP_
