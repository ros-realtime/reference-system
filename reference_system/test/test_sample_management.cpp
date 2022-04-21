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

#include "reference_system/sample_management.hpp"
#include "reference_system/msg_types.hpp"


TEST(test_sample_management, set_benchmark_mode) {
  ::testing::Test::RecordProperty("TEST_ID", "41781862-6454-4d85-a3ee-bf82b1872763");
  EXPECT_FALSE(is_in_benchmark_mode());
  set_benchmark_mode(true);
  EXPECT_TRUE(is_in_benchmark_mode());
  set_benchmark_mode(false);
  EXPECT_FALSE(is_in_benchmark_mode());
}

TEST(test_sample_management, sample_helpers) {
  ::testing::Test::RecordProperty("TEST_ID", "6245be8e-3050-432f-9f61-a73dd33ff2c4");
  message_t sample;
  std::string node_name = "test_node";
  uint32_t sequence_number = 10;
  uint32_t dropped_samples = 5;
  uint64_t timestamp = 8675309;
  set_sample(node_name, sequence_number, dropped_samples, timestamp, sample);
  // EXPECT_EQ(sample.stats[0].node_name.data(), node_name.data());
  // see reference_interfaces for more details
  // 4kb msg = 4032 = 63 * 64 bytes, 64 bytes = TransmissionStats length
  EXPECT_EQ(sample.stats.size(), 63u);
  EXPECT_EQ(sample.stats[0].sequence_number, sequence_number);
  EXPECT_EQ(sample.stats[0].dropped_samples, dropped_samples);
  EXPECT_EQ(sample.stats[0].timestamp, timestamp);

  auto retrieved_stamp = get_sample_timestamp(&sample);
  EXPECT_EQ(retrieved_stamp, timestamp);

  auto retrieved_sequence_number = get_sample_sequence_number(&sample);
  EXPECT_EQ(retrieved_sequence_number, sequence_number);

  auto retrieved_dropped_samples =
    get_missed_samples_and_update_seq_nr(&sample, sequence_number);
  // should be zero based on sequence number
  EXPECT_EQ(retrieved_dropped_samples, 0u);
}

TEST(test_sample_management, statistic_value_struct) {
  auto stats = statistic_value_t();

  stats.suffix = "the_suffix";
  // simulate multiple messages coming in
  stats.set(1);
  stats.set(4);
  stats.set(5);
  stats.set(7);

  EXPECT_EQ(stats.average, 4.25);
  EXPECT_EQ(stats.deviation, 2.5);
  EXPECT_EQ(stats.min, 1u);
  EXPECT_EQ(stats.max, 7u);
  EXPECT_EQ(stats.current, 7u);
  EXPECT_EQ(stats.total_number, 4u);
  EXPECT_EQ(stats.suffix, "the_suffix");
  EXPECT_EQ(stats.adjustment, 0.0);
}

TEST(test_sample_management, sample_statistic_struct) {
  auto stats = sample_statistic_t();
  stats.timepoint_of_first_received_sample = 1;
  stats.previous_behavior_planner_sequence = 2;
  stats.previous_behavior_planner_time_stamp = 3;

  EXPECT_EQ(stats.timepoint_of_first_received_sample, uint64_t(1));
  EXPECT_EQ(stats.previous_behavior_planner_sequence, uint32_t(2));
  EXPECT_EQ(stats.previous_behavior_planner_time_stamp, uint64_t(3));
}

TEST(test_sample_management, print_sample_path) {
  message_t sample;

  std::string node_name = "test_node";

  uint32_t sample_size = 105;
  // TODO(evan.flynn): add test for operator<< print function used within print_sample_path
  set_benchmark_mode(false);
  uint64_t timestamp = 1;
  for (uint32_t i = 0; i < sample_size; i++) {
    set_sample(node_name, i, 0, timestamp, sample);
    timestamp += 1;
  }
  // message sample size will always be 63 if message type is set to 4kb
  // see reference_interfaces package for more details
  EXPECT_EQ(sample.size, 63u);
  print_sample_path("test_node", uint32_t(1), &sample);
}

TEST(test_sample_management, set_sample_terminates_node_name) {
  message_t sample;
  const std::string short_name("name_with_few_characters");
  const std::string long_name(sample.stats[0].node_name.size(), 'A');

  // Fill the stats buffer with '%'-characters. These characters must no
  // longer be visible in the resulting buffer
  sample.size = 0;
  sample.stats[0].node_name.fill('%');
  set_sample(short_name, 1, 0, 0, sample);
  EXPECT_STREQ(
    short_name.c_str(),
    reinterpret_cast<const char *>(sample.stats[0].node_name.data()));

  sample.size = 0;
  sample.stats[0].node_name.fill('%');
  set_sample(long_name, 1, 0, 0, sample);
  ASSERT_EQ(sample.stats[0].node_name.back(), '\0');
  auto long_name_without_last_char = long_name.substr(0, long_name.size() - 1);
  EXPECT_STREQ(
    long_name_without_last_char.c_str(),
    reinterpret_cast<const char *>(sample.stats[0].node_name.data()));
}
