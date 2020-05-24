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
