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

#include <chrono>
#include <cmath>
#include <vector>

uint64_t number_cruncher(const uint64_t maximum_number)
{
  uint64_t number_of_primes = 0;
  bool has_timeout_occurred = false;
  for (uint64_t i = 3; i < maximum_number; ++i) {
    uint64_t rootOfI = static_cast<uint64_t>(std::sqrt(i));
    bool is_prime = true;
    for (uint64_t n = 2; n < rootOfI; ++n) {
      if (i % n == 0) {
        is_prime = false;
        break;
      }
    }

    if (is_prime) {
      ++number_of_primes;
    }
  }
  return number_of_primes;
}

#endif  // REFERENCE_SYSTEM__NUMBER_CRUNCHER_HPP_
