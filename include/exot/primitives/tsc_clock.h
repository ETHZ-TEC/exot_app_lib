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
 * @file primitives/tsc_clock.h
 * @author     Bruno Klopott
 * @brief      A clock with std::chrono-like interface that explicitly uses the
 *             time stamp counter.
 */

#pragma once

#ifdef __x86_64__

#include <chrono>   // for time_point and duration
#include <cstdint>  // for uint*
#include <thread>   // for this_thread

#include <exot/primitives/tsc.h>    // for tsc::start, tsc::stop
#include <exot/utilities/timing.h>  // for nanosleep

namespace exot::primitives {

inline namespace x86_64 {
// class platform_specific_tsc {
//  public:
//   typedef std::chrono::duration<std::chrono::nanoseconds::rep,
//                                 std::ratio<1, 1'600'000'000>>
//       internal_duration;
// };

/**
 * @brief      Clock that uses the time stamp counter explicitly
 * @details    The clock uses the same interface as other clock from the C++
 *             STL. It can be used as a drop-in replacement whenever a standard
 *             clock interface is required.
 *
 *             However, due to the need to compute the TSC frequency at runtime,
 *             there is a small caveat. To maintain the same static interface as
 *             other clock types, calibration has to be performed during the
 *             first call to the `now()` function. Since the values for `init`
 *             and `frequency` are declared static, they will be executed
 *             exactly once. As soon as their values are set, the calls to
 *             `tsc_get_frequency()` will not be repeated. In some cases this
 *             one-off overhead might be acceptable, since all chrono functions,
 *             duration computations and casts work with the time stamp counter.
 */
template <typename T = SimpleTSC,
          typename   = std::enable_if_t<is_time_stamp_counter_v<T>>>
class tsc_clock {
 public:
  typedef std::chrono::nanoseconds duration;
  typedef duration::rep rep;
  typedef duration::period period;
  typedef std::chrono::time_point<tsc_clock> time_point;
  static constexpr bool is_steady        = true;
  static constexpr auto overhead_sleep   = std::chrono::milliseconds{100};
  static constexpr unsigned overhead_rep = 10;

  using tsc = T;

  /**
   * @brief      Gets the current time point
   * @details    This is the only static function that an STL-like clock has to
   *             provide.
   *
   * @return     The current time point
   */
  static time_point now() noexcept {
    /* Initialise the time stamp counter, called only once for all uses of the
     * `tsc_clock`. Static local variables have static storage duration and are
     * initialised the first time the control flow passes their declarations.
     *
     * Thanks to that the functions tsc_* do not have to be static. */
    static auto init = tsc::start();
    /* Get the time stamp counter frequency, called only once for all uses of
     * the `tsc_clock`. */
    static auto frequency = get_tsc_frequency();
    /* The multiplication factor used to convert cycles to the base period type,
     * also a static local object. */
    static long double multiplier =
        static_cast<long double>(period::den) / frequency;

    /* The static local variables above constitute the initialisation of the
     * tsc_clock.
     *
     * As a matter of fact, the implementation of the <chrono> header of the
     * standard library often relies on such a pattern, e.g. in the
     * implementation of the steady_clock::now(): code{static FP fp =
     * init_steady_clock();}. */

    /* If tsc_clock inherits from `platform_specific_tsc`, the internal duration
     * with a custom ratio can be used, and the time point can be obtained by
     * casting the internal duration to the public duration (nanoseconds). */

    // auto internal = tsc_clock::internal_duration{cycles};
    // auto duration =
    // std::chrono::duration_cast<tsc_clock::duration>(internal);

    /*! Access to the current time stamp counter value. */
    auto cycles = tsc::stop();
    /* Convert and cast the cycle count to base duration type. */
    auto _ = static_cast<decltype(cycles)>(multiplier * cycles);

    /* Form the time point. */
    return time_point{duration{_}};
  }

  /**
   * @brief      Get the sleep duration/overhead in cycles
   *
   * @param[in]  duration_  The sleep duration
   *
   * @return     Number of cycles taken by the sleep call
   */
  static std::uint64_t get_sleep_cycle_count(duration duration_) {
    auto start = tsc::start();
    exot::utilities::nanosleep(duration_);
    return (tsc::stop() - start);
  }

  /**
   * @brief      Get the average sleep overhead in cycles
   *
   * @param[in]  duration  The sleep duration
   * @param[in]  rep       The number of test repetitions
   *
   * @return     The average number of cycles for the sleep call
   */
  static std::uint64_t get_sleep_overhead(duration duration_,
                                          unsigned int rep) {
    std::uint64_t accum{0};
    unsigned int count{rep};
    while (count--) accum += get_sleep_cycle_count(duration_);
    return accum / rep;
  }

  /**
   * @brief      Calculate the time stamp counter frequency
   *
   * @return     The time stamp counter frequency in Hz
   */
  static rep get_tsc_frequency() {
    /* The average overhead of just the call to `sleep_for` with zero sleep
     * duration. */
    auto sleep_ovh = get_sleep_overhead(duration{0}, 100);
    /* The average cycle count of a call to sleep for `sleep_time`. */
    auto sleep_cycles = get_sleep_overhead(overhead_sleep, overhead_rep);
    /* The sleep time casted to clock's base duration. */
    static auto sleep_ns = std::chrono::duration_cast<duration>(overhead_sleep);

    /* `tsc_clock::duration::period::den` is used just in case the clock period
     * is not in nanoseconds. */
    return period::den * (sleep_cycles - sleep_ovh) / sleep_ns.count();
  }
};

}  // namespace x86_64

}  // namespace exot::primitives

#endif
