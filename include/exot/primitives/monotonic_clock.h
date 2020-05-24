// Copyright (c) 2015-2020, Swiss Federal Institute of Technology (ETH Zurich)
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
// 
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// 
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
/**
 * @file primitives/monotonic_clock.h
 * @author     Bruno Klopott
 * @brief      Getter, clock and counter for the Unix monotonic clock.
 */

#pragma once

#if defined(__unix__) || (defined(__APPLE__) && defined(__MACH__))

#include <time.h>  // for nanosleep, timespec, clock_gettime, ...

#include <chrono>
#include <cstdint>

#include <exot/primitives/tsc.h>

namespace exot::primitives {

inline __attribute__((always_inline)) std::uint64_t get_monotonic_time() {
  struct ::timespec _timespec;
  ::clock_gettime(CLOCK_MONOTONIC, &_timespec);
  return _timespec.tv_sec * 1'000'000'000ull + _timespec.tv_nsec;
}

struct monotonic_clock {
  typedef std::chrono::nanoseconds duration;
  typedef duration::rep rep;
  typedef duration::period period;
  typedef std::chrono::time_point<monotonic_clock> time_point;
  static constexpr bool is_steady = true;

  static inline time_point now() noexcept {
    return time_point{duration{get_monotonic_time()}};
  }
};

struct MonotonicCounter : TimeStampCounter {
  static inline __attribute__((always_inline)) std::uint64_t start() {
    return get_monotonic_time();
  }

  static inline __attribute__((always_inline)) std::uint64_t stop() {
    return get_monotonic_time();
  }
};

}  // namespace exot::primitives

#endif
