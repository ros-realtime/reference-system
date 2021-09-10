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
#ifndef REFERENCE_SYSTEM__NUMBER_CRUNCHER_HPP_
#define REFERENCE_SYSTEM__NUMBER_CRUNCHER_HPP_
#pragma once

#include <chrono>
#include <cmath>
#include <vector>

std::vector<uint64_t> number_cruncher(const std::chrono::nanoseconds & timeout)
{
  // cause heavy CPU load by finding some primes
  std::vector<uint64_t> primes{2};
  // allocate memory beforehand so that we put only load on the CPU
  primes.reserve(1024 * 1024);
  bool has_timeout_occurred = false;
  auto current_time = [] {
      return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch())
             .count();
    };
  int64_t start_time = current_time();
  for (uint64_t i = 3; !has_timeout_occurred; ++i) {
    uint64_t rootOfI = std::sqrt(i);
    for (auto p : primes) {
      if (current_time() - start_time > timeout.count()) {
        has_timeout_occurred = true;
      }
      if (p > rootOfI) {
        primes.emplace_back(i);
        break;
      } else if (i % p == 0) {
        break;
      }
    }
  }
  return primes;
}
#endif  // REFERENCE_SYSTEM__NUMBER_CRUNCHER_HPP_
