/**
 * @file utilities/timing_source.h
 * @author     Bruno Klopott
 * @brief      Single point configuration for special timing facilities.
 */

#pragma once

#include <chrono>

#include <exot/primitives/fenced_clock.h>
#include <exot/primitives/monotonic_clock.h>
#include <exot/primitives/perf_clock.h>
#include <exot/primitives/tsc.h>

#include <exot/utilities/timing.h>

namespace exot::utilities {

/**
 * @brief Enumeration for timing serialisation type
 */
enum class TimingFenceType : unsigned char {
  Atomic = 0,
  Weak   = 1,
  Strong = 2,
  None   = 3,
};

/**
 * @brief Enumeration for timing source
 */
enum class TimingSourceType : unsigned char {
  SteadyClock                = 0,  // std::steady_clock
  MonotonicCounter           = 1,  // MonotonicCounter
  MonotonicClock             = 2,  // monotonic_clock
  TimeStampCounter           = 3,  // SimpleTSC
  HardwarePerformanceCounter = 4,  // perf using PERF_COUNT_HW_CPU_CYCLES
  SoftwarePerformanceCounter = 5,  // perf using PERF_COUNT_SW_CPU_CYCLES
};

#if !defined(EXOT_TIME_FENCE)
#define EXOT_TIME_FENCE 0  //! Defines the default serialisation type
#endif

#if !defined(EXOT_TIME_SOURCE)
#define EXOT_TIME_SOURCE 0  //! Defines the default timing source
#endif

// clang-format off
#ifndef CONFIGURATION_MESSAGE_PRINTED
#define CONFIGURATION_MESSAGE_PRINTED
#if EXOT_TIME_FENCE == 0
#pragma message "EXOT_TIME_FENCE: Chose TimingFenceType::Atomic"
#elif EXOT_TIME_FENCE == 1
#pragma message "EXOT_TIME_FENCE: Chose TimingFenceType::Weak"
#elif EXOT_TIME_FENCE == 2
#pragma message "EXOT_TIME_FENCE: Chose TimingFenceType::Strong"
#elif EXOT_TIME_FENCE == 3
#pragma message "EXOT_TIME_FENCE: Chose TimingFenceType::None"
else
#error "EXOT_TIME_FENCE has no or wrong value!"
#endif

#if EXOT_TIME_SOURCE == 0
#pragma message "EXOT_TIME_SOURCE: Chose TimingSourceType::SteadyClock"
#elif EXOT_TIME_SOURCE == 1
#pragma message "EXOT_TIME_SOURCE: Chose TimingSourceType::MonotonicCounter"
#elif EXOT_TIME_SOURCE == 2
#pragma message "EXOT_TIME_SOURCE: Chose TimingSourceType::MonotonicClock"
#elif EXOT_TIME_SOURCE == 3
#pragma message "EXOT_TIME_SOURCE: Chose TimingSourceType::TimeStampCounter"
#elif EXOT_TIME_SOURCE == 4
#pragma message "EXOT_TIME_SOURCE: Chose TimingSourceType::HardwarePerformanceCounter"
#elif EXOT_TIME_SOURCE == 5
#pragma message "EXOT_TIME_SOURCE: Chose TimingSourceType::SoftwarePerformanceCounter"
else
#error "EXOT_TIME_SOURCE has no or wrong value!"
#endif
#endif
// clang-format on

/**
 * @brief The default serialisation type
 */
inline constexpr TimingFenceType EXOT_TIME_FENCE_TYPE =
    static_cast<TimingFenceType>(EXOT_TIME_FENCE);

/**
 * @brief The default timing source type
 */
inline constexpr TimingSourceType EXOT_TIME_SOURCE_TYPE =
    static_cast<TimingSourceType>(EXOT_TIME_SOURCE);

/**
 * @brief   The serialised time source
 *
 * @tparam  T          A clock or counter type
 * @tparam  Fence      The serialisation type
 * @tparam  <unnamed>  Template helper
 */
template <typename T, TimingFenceType Fence, typename = void>
struct serialised_time_source_t;

template <typename Clock>
struct serialised_time_source_t<
    Clock, TimingFenceType::Atomic,
    std::enable_if_t<exot::utilities::is_clock_v<Clock>>>
    : public exot::primitives::fenced_clock<Clock> {};

template <typename Clock>
struct serialised_time_source_t<
    Clock, TimingFenceType::Weak,
    std::enable_if_t<exot::utilities::is_clock_v<Clock>>>
    : public exot::primitives::__fenced_clock<Clock> {};

template <typename Clock>
struct serialised_time_source_t<
    Clock, TimingFenceType::Strong,
    std::enable_if_t<exot::utilities::is_clock_v<Clock>>>
    : public exot::primitives::__strong_fenced_clock<Clock> {};

template <typename Clock>
struct serialised_time_source_t<
    Clock, TimingFenceType::None,
    std::enable_if_t<exot::utilities::is_clock_v<Clock>>>
    : public exot::primitives::plain_clock<Clock> {};

template <typename Counter>
struct serialised_time_source_t<
    Counter, TimingFenceType::Atomic,
    std::enable_if_t<exot::primitives::is_time_stamp_counter_v<Counter>>>
    : public exot::primitives::fenced_tsc<Counter> {};

template <typename Counter>
struct serialised_time_source_t<
    Counter, TimingFenceType::Weak,
    std::enable_if_t<exot::primitives::is_time_stamp_counter_v<Counter>>>
    : public exot::primitives::__fenced_tsc<Counter> {};

template <typename Counter>
struct serialised_time_source_t<
    Counter, TimingFenceType::Strong,
    std::enable_if_t<exot::primitives::is_time_stamp_counter_v<Counter>>>
    : public exot::primitives::__strong_fenced_tsc<Counter> {};

template <typename Counter>
struct serialised_time_source_t<
    Counter, TimingFenceType::None,
    std::enable_if_t<exot::primitives::is_time_stamp_counter_v<Counter>>>
    : public exot::primitives::plain_tsc<Counter> {};

/**
 * @brief   The time source type, a clock or a counter
 *
 * @tparam  Source  The timing source type
 */
template <TimingSourceType Source>
struct time_source_t;

template <>
struct time_source_t<TimingSourceType::SteadyClock>
    : public std::chrono::steady_clock {};

template <>
struct time_source_t<TimingSourceType::MonotonicCounter>
    : public exot::primitives::MonotonicCounter {};

template <>
struct time_source_t<TimingSourceType::MonotonicClock>
    : public exot::primitives::monotonic_clock {};

#if defined(__x86_64__)
template <>
struct time_source_t<TimingSourceType::TimeStampCounter>
    : public exot::primitives::SimpleTSC {};
#endif

template <>
struct time_source_t<TimingSourceType::HardwarePerformanceCounter>
    : public exot::primitives::perf_clock {};

template <>
struct time_source_t<TimingSourceType::SoftwarePerformanceCounter>
    : public exot::primitives::perf_sw_clock {};

/**
 * @brief The default time source
 */
using default_time_source = time_source_t<EXOT_TIME_SOURCE_TYPE>;

/**
 * @brief The default serialised time source
 */
using default_serialised_time_source =
    serialised_time_source_t<default_time_source, EXOT_TIME_FENCE_TYPE>;

/**
 * @brief   The default timeing facility
 * @note    Wrapper of the "timeit" utility defined in
 * <exot/utilities/timing.h>
 *
 * @tparam  Args  The forwarded arguments types
 * @param   args  The forwarded arguments
 *
 * @return  The time measurement output in clock-specific type
 */
template <typename... Args>
inline auto default_timing_facility(Args&&... args) {
  if constexpr (exot::primitives::is_time_stamp_counter_v<
                    typename default_serialised_time_source::
                        underlying_timing_source>) {
    return timeit<default_serialised_time_source>(std::forward<Args>(args)...);
  } else {
    return timeit<default_serialised_time_source>(std::forward<Args>(args)...)
        .count();
  }
}

}  // namespace exot::utilities