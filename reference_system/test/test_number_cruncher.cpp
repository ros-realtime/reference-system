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

#include "reference_system/number_cruncher.hpp"


TEST(test_number_cruncher, number_cruncher) {
  auto primes = number_cruncher(10);
  // 2, 3, 5, 7
  EXPECT_EQ(primes, 4);

  primes = number_cruncher(20);
  // 11, 13, 17, 19
  EXPECT_EQ(primes, 8);

  primes = number_cruncher(30);
  // 23, 29
  EXPECT_EQ(primes, 10);
}

TEST(test_number_cruncher, crunch_time) {
  auto expected_fast = get_crunch_time_in_ms(100);
  auto expected_slow = get_crunch_time_in_ms(65536);
  // lower maximum number should result in faster crunch times
  EXPECT_LT(expected_fast, expected_slow);
}
