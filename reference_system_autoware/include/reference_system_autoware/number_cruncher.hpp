#pragma once

#include <chrono>
#include <cmath>
#include <vector>

std::vector<uint64_t> number_cruncher(const std::chrono::nanoseconds& timeout) {
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
            if (current_time() - start_time > timeout.count())
                has_timeout_occurred = true;
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
