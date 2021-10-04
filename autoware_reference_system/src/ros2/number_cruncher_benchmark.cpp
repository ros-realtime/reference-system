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
//
#include "reference_system/number_cruncher.hpp"

#include <iostream>
#include <iomanip>

long double get_crunch_time_in_ms(const uint64_t maximum_number) {
    auto start = std::chrono::system_clock::now();
    number_cruncher(maximum_number);
    auto stop = std::chrono::system_clock::now();
    return static_cast<long double>(
        std::chrono::nanoseconds(stop - start).count() / 1000000.0);
}
int main() {
    long double crunch_time = 0.0;
    std::cout << "maximum_number     run time" << std::endl;
    for (uint64_t i = 10; crunch_time < 1000.0; i *= 10) {
        crunch_time = get_crunch_time_in_ms(i);
        std::cout << std::setfill(' ') << std::setw(12) << i << "       "
                  << crunch_time << "ms" << std::endl;
    }
}
