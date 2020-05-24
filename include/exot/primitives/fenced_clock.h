/**
 * @file primitives/fenced_clock.h
 * @author     Bruno Klopott
 * @brief      Wrappers for making clock sources serialised.
 */

#pragma once

#include <atomic>
#include <chrono>

#include <exot/primitives/ordering.h>
#include <exot/utilities/types.h>

#include <exot/primitives/tsc.h>

namespace exot::primitives {

template <typename Clock,
          typename = std::enable_if_t<exot::utilities::is_clock_v<Clock>>>
struct plain_clock : public Clock {
  using underlying_timing_source = Clock;
};

template <typename Clock,
          typename = std::enable_if_t<exot::utilities::is_clock_v<Clock>>>
struct fenced_clock : public Clock {
  using underlying_timing_source = Clock;

  /**
   * @brief      Wrapper of Clock's now() function with C++ memory ordering
   *             instructions
   *
   * @return     The Clock's time point
   */
  static inline auto now() {
    std::atomic_thread_fence(std::memory_order_acquire);
    auto _ = Clock::now();
    std::atomic_thread_fence(std::memory_order_release);
    return _;
  }
};

using fenced_steady_clock = fenced_clock<std::chrono::steady_clock>;

template <typename Clock,
          typename = std::enable_if_t<exot::utilities::is_clock_v<Clock>>>
struct __fenced_clock : public Clock {
  using underlying_timing_source = Clock;

  static inline auto now() {
    load_fence();
    auto _ = Clock::now();
    memory_fence();
    return _;
  }
};

using __fenced_steady_clock = __fenced_clock<std::chrono::steady_clock>;

template <typename Clock,
          typename = std::enable_if_t<exot::utilities::is_clock_v<Clock>>>
struct __strong_fenced_clock : public Clock {
  using underlying_timing_source = Clock;

  static inline auto now() {
    full_fence();
    auto _ = Clock::now();
    full_fence();
    return _;
  }
};

using __strong_fenced_steady_clock =
    __strong_fenced_clock<std::chrono::steady_clock>;

template <typename Counter,
          typename = std::enable_if_t<is_time_stamp_counter_v<Counter>>>
struct plain_tsc : public Counter {
  using underlying_timing_source = Counter;
};

template <typename Counter,
          typename = std::enable_if_t<is_time_stamp_counter_v<Counter>>>
struct fenced_tsc {
  using underlying_timing_source = Counter;

  static inline __attribute__((always_inline)) std::uint64_t start() {
    std::atomic_thread_fence(std::memory_order_acquire);
    auto _ = Counter::start();
    std::atomic_thread_fence(std::memory_order_release);
    return _;
  }
  static inline __attribute__((always_inline)) std::uint64_t stop() {
    std::atomic_thread_fence(std::memory_order_acquire);
    auto _ = Counter::stop();
    std::atomic_thread_fence(std::memory_order_release);
    return _;
  }
};

template <typename Counter,
          typename = std::enable_if_t<is_time_stamp_counter_v<Counter>>>
struct __fenced_tsc {
  using underlying_timing_source = Counter;

  static inline __attribute__((always_inline)) std::uint64_t start() {
    load_fence();
    auto _ = Counter::start();
    memory_fence();
    return _;
  }
  static inline __attribute__((always_inline)) std::uint64_t stop() {
    load_fence();
    auto _ = Counter::stop();
    memory_fence();
    return _;
  }
};

template <typename Counter,
          typename = std::enable_if_t<is_time_stamp_counter_v<Counter>>>
struct __strong_fenced_tsc {
  using underlying_timing_source = Counter;

  static inline __attribute__((always_inline)) std::uint64_t start() {
    full_fence();
    auto _ = Counter::start();
    full_fence();
    return _;
  }
  static inline __attribute__((always_inline)) std::uint64_t stop() {
    full_fence();
    auto _ = Counter::stop();
    full_fence();
    return _;
  }
};

}  // namespace exot::primitives
