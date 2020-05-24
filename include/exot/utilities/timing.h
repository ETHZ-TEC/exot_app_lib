/**
 * @file utilities/timing.h
 * @author     Bruno Klopott
 * @brief      Timing utilities, including the TimeKeeper template for running
 *             timing-critical code.
 */

#pragma once

#include <algorithm>    // for std::generate
#include <array>        // for fixed-size arrays
#include <chrono>       // for clocks and durations
#include <cmath>        // for std::sqrt
#include <ctime>        // for std::tm and std::*time
#include <numeric>      // for std::inner_product, std::accumulate
#include <string>       // for std::string
#include <thread>       // for std::this_thread::sleep_for
#include <type_traits>  // for true/false_type and declval
#include <vector>       // for variable-size arrays

/**
 * On POSIX platforms, provide an STL-like wrapper for `nanosleep`.
 */
#if defined(__unix__) || (defined(__APPLE__) && defined(__MACH__))
#include <time.h>  // for nanosleep and timespec
#endif

#include <fmt/format.h>     // for string formatting
#include <spdlog/spdlog.h>  // for logging

#include <exot/utilities/formatting.h>  // for duration_unit
#include <exot/utilities/helpers.h>     // for do_not_optimise
#include <exot/utilities/types.h>       // for type traits

#include <exot/primitives/tsc.h>

namespace exot::utilities {

/**
 * @brief      Time the execution of a callable using timestamp counters
 *
 * @param      callable   The callable
 * @param      args       The arguments to the callable
 *
 * @tparam     Counter    A TSC (must derive from TimeStampCounter)
 * @tparam     Callable   A callable type
 * @tparam     Args       Argument types to the callable
 * @tparam     <unnamed>  Template helper
 *
 * @return     The execution duration in TSC-specific units
 */
template <typename Counter, typename Callable, typename... Args,
          typename = std::enable_if_t<
              exot::primitives::is_time_stamp_counter_v<Counter>>>
auto timeit(Callable&& callable, Args&&... args) {
  auto start = Counter::start();
  callable(std::forward<Args>(args)...);
  return Counter::stop() - start;
}

/**
 * @brief      Time the execution of a callable using any std::chrono-like clock
 *
 * @param      callable   The callable
 * @param      args       The arguments to the callable
 *
 * @tparam     Clock      A Clock type, e.g. std::chrono::steady_clock
 * @tparam     Callable   A callable type
 * @tparam     Args       Argument types to the callable
 * @tparam     <unnamed>  Template helper
 *
 * @return     The execution duration as a duration, e.g. chrono::nanoseconds
 */
template <typename Clock, typename Callable, typename... Args,
          typename = std::enable_if_t<is_clock_v<Clock>>>
auto timeit(Callable&& callable, Args&&... args) -> typename Clock::duration {
  auto start = Clock::now();
  callable(std::forward<Args>(args)...);
  return Clock::now() - start;
}

namespace details {

/**
 * @brief      Functor to estimate the overhead of the sleeping function.
 *
 * @tparam     Clock     Evaluated clock type, e.g. std::chrono::steady_clock;
 */
template <typename Clock>
struct sleep_overhead {
  using clock_type    = Clock;
  using duration_type = typename Clock::duration;

  /**
   * @brief      Constructor that sets the sleep duration
   *
   * @param[in]  duration  The sleep duration
   */
  explicit sleep_overhead(duration_type duration) : duration{duration} {};

  /**
   * @brief      The default constructor, estimate the overhead of calling the
   *             sleep function with zero sleep duration
   */
  sleep_overhead() : sleep_overhead(duration_type{0}){};

  duration_type operator()() {
    auto start{clock_type::now()};
    std::this_thread::sleep_for(duration);
    return {clock_type::now() - start};
  }

 private:
  duration_type duration{0};
};

/**
 * @brief      Functor to estimate the overhead of the basic timing primitives.
 *
 * @tparam     Clock     Evaluated clock type, e.g. std::chrono::steady_clock;
 */
template <typename Clock>
struct timing_overhead {
  using clock_type    = Clock;
  using duration_type = typename clock_type::duration;

  /**
   * @return Time difference between the consecutive calls to now().
   */
  duration_type operator()() {
    auto start(clock_type::now());
    return (clock_type::now() - start);
  };
};

/**
 * @brief      Functor to estimate the mean overhead for basic timing
 *             primitives.
 *
 * @tparam     Clock     Evaluated clock type, e.g. std::chrono::steady_clock;
 * @tparam     Overhead  The particular overhead functor
 */
template <typename Clock, template <typename> typename Overhead>
struct get_overhead {
  using clock_type    = Clock;
  using duration_type = typename clock_type::duration;
  using overhead      = Overhead<clock_type>;

  /**
   * @brief      Repetitively calls timing_overhead.
   *
   * @param[in]  iterations  Number of repeated calls to timing_overhead.
   * @return     Mean timing overhead over all iterations.
   */
  duration_type operator()(unsigned int iterations) {
    std::vector<duration_type> overheads(iterations);

    std::generate(overheads.begin(), overheads.end(), overhead());
    return std::accumulate(overheads.begin(), overheads.end(),
                           (duration_type(0))) /
           overheads.size();
  };
};

/**
 * @brief      Computes mean and standard deviation from a vector of
 *             std::chrono::duration.
 *
 * @param[in]  input              Input vector of InputGranularity objects.
 *
 * @tparam     InputGranularity   Input duration granularity used in the vector,
 *                                e.g. `std::chrono::nanoseconds`;
 * @tparam     OutputGranularity  Output duration granularity;
 *
 * @return     Tuple holding computed mean and standard deviation.
 */
template <typename InputGranularity, typename OutputGranularity>
auto chrono_statistics(const std::vector<InputGranularity>& input) {
  auto mean =
      std::accumulate(input.begin(), input.end(), (InputGranularity(0))) /
      input.size();

  /* Calculate the mean differences */
  std::vector<typename InputGranularity::rep> deviations(input.size());
  std::transform(input.begin(), input.end(), deviations.begin(),
                 [mean](InputGranularity in) { return (in - mean).count(); });

  /* The accumulator {a} in inner_product calculates: a += *iterator1 *
   * *iterator2, starts with 0.0. Computes the squared sum(x - mean)(x - mean)
   * in variance computation. */
  auto squared_sum = std::inner_product(deviations.begin(), deviations.end(),
                                        deviations.begin(), 0.0);

  auto standard_deviation_ = std::sqrt(squared_sum / input.size());

  /* Average interval */
  OutputGranularity average =
      std::chrono::duration_cast<OutputGranularity>(mean);

  /* Standard deviation */
  std::chrono::duration<double, typename InputGranularity::period> tmp(
      standard_deviation_);
  OutputGranularity standard_deviation =
      std::chrono::duration_cast<OutputGranularity>(tmp);

  return std::array<OutputGranularity, 2>{average, standard_deviation};
}

}  // namespace details

#ifndef TimeKeeper_OFFSET_RETURNS
#define TimeKeeper_OFFSET_RETURNS 0
#endif

namespace details {

struct TimeKeeperOptions {
  static constexpr bool offset_returns =
      static_cast<bool>(TimeKeeper_OFFSET_RETURNS);
};

inline constexpr TimeKeeperOptions TIME_KEEPER_OPTIONS;
}  // namespace details

/**
 * @brief      Class used for keeping track of sleep time and offsets.
 * @details    The time keeper provides an interface for sleeping with offset
 *             and overhead accounting, can use different clock sources and
 *             sleep functions, and can report interval, offset, and
 *             differential statistics of sleep durations.
 *
 * @tparam     Clock           A clock type, which provides a `now()` method,
 *                             e.g. `std::chrono::steady_clock`
 * @tparam     EnableLogging   A boolean value to indicate use internal offset
 *                             logging
 * @tparam     RepGranularity  Chrono duration granularity used for time
 *                             reporting
 * @tparam     Granularity     Chrono duration granularity used for time
 *                             keeping, e.g. `std::chrono::nanoseconds`
 */
template <typename Clock = std::chrono::steady_clock, bool EnableLogging = true,
          typename RepGranularity = std::chrono::microseconds,
          typename Granularity    = std::chrono::nanoseconds>
class TimeKeeper {
 public:
  using clock_type            = Clock;
  using chrono_granularity    = Granularity;
  using reporting_granularity = RepGranularity;

  using sleep_func_type   = std::function<void(const chrono_granularity&)>;
  using sleep_func_rep    = typename chrono_granularity::rep;
  using sleep_func_period = typename chrono_granularity::period;

  /**
   * Default values for reserved log size, overhead iterations, and minimum
   * sleep duration.
   *
   * The storage size of most duration types is equal or less than the size of
   * `unsigned long`, which is 8 bytes. A pre-reserved vector log size of
   * 10000000 amounts to 80 MB of RAM used for keeping the log. While the log
   * could be smaller, depending on the duration of an experiment, one might
   * experience a performance drop due to the need to reallocate storage space
   * for the vector.
   */
  static const unsigned int default_log_size_to_reserve{10000000};
  static const unsigned int default_overhead_calculations{1000};
  static const typename chrono_granularity::rep default_minimum_percentage{0};

  /**
   * @brief      Constructs the object with all options.
   * @details    In order to allow the use of different sleep functions (e.g.
   *             `nanosleep`, `std::this_thread::sleep_for`,
   *             `boost::this_fiber::sleep_for`, etc.) the constructor takes a
   *             pointer to that function, instead of using the older template
   *             parameter.
   *
   * @param[in]  logger               A shared pointer to application logger
   * @param[in]  sleep_function       The address of a sleep function, by
   *                                  default uses the
   *                                  `std::this_thread::sleep_for` function.
   * @param[in]  minimum_percentage   The minimum percentage of the passed sleep
   *                                  duration that will be passed to the
   *                                  sleeping function. The value is used in
   *                                  the call to `std::max`, which result is
   *                                  passed to the actual sleeping function.
   * @param[in]  log_size_to_reserve  The reserved vector size for logging
   *                                  desired, actual, and offset durations
   * @param[in]  overhead_iterations  The number of iterations used for
   *                                  calculating the offset of a sequence of
   *                                  calls to `clock_type::now()` and the
   *                                  necessary subtraction used to obtain the
   *                                  duration between the calls.
   */
  explicit TimeKeeper(
      std::shared_ptr<spdlog::logger> logger = nullptr,
      sleep_func_type sleep_function =
          &std::this_thread::sleep_for<sleep_func_rep, sleep_func_period>,
      typename chrono_granularity::rep minimum_percentage =
          default_minimum_percentage,
      unsigned int log_size_to_reserve = default_log_size_to_reserve,
      unsigned int overhead_iterations = default_overhead_calculations)
      : sleep_function_(sleep_function),
        logger_{std::move(logger)},
        minimum_percentage_{minimum_percentage} {
    logger_ = logger_ ? logger_
                      : (spdlog::get("log") ? spdlog::get("log")
                                            : spdlog::default_logger());
    bootstrap(overhead_iterations, log_size_to_reserve);
  };

  /**
   * @brief      Initialises the time keeping reference time.
   * @details    There is no need to keep a `start_` variable, because
   *             `desired_` is incremented with every call to the `sleep()`
   *             method.
   */
  inline void begin() {
    desired_ = clock_type::now();
    now_     = desired_;

    if constexpr (EnableLogging) {
      actual_log_.push_back(now_);
      desired_log_.push_back(desired_);
      offsets_log_.emplace_back(0);
    }

    SPDLOG_LOGGER_TRACE(logger_, "[TimeKeeper] begin: {}",
                        desired_.time_since_epoch().count());
  };

  inline auto now() const { return now_; }

  /**
   * @brief      Updates the offset used to compensate for the mismatch between
   *             the desired and the actual time point.
   */
  auto update_offset() {
    now_    = clock_type::now();
    offset_ = now_ - desired_;

    SPDLOG_LOGGER_TRACE(logger_,
                        "[TimeKeeper] now: {}, desired: {}, offset: {}",
                        now_.time_since_epoch().count(),
                        desired_.time_since_epoch().count(), offset_.count());

    /* Push the actual, desired, and the resulting offset duration. */
    if constexpr (EnableLogging) {
      actual_log_.push_back(now_);
      desired_log_.push_back(desired_);
      offsets_log_.push_back(offset_);
    }

    if constexpr (details::TIME_KEEPER_OPTIONS.offset_returns) {
      return std::array<reporting_granularity, 3>{
          std::chrono::duration_cast<reporting_granularity>(
              now_.time_since_epoch()),
          std::chrono::duration_cast<reporting_granularity>(
              desired_.time_since_epoch()),
          std::chrono::duration_cast<reporting_granularity>(offset_)};
    }
  };

  /**
   * @brief      Sleeps for the specified duration, taking account of the
   *             current offset.
   * @details    Uses the sleep function provided in the time keeper
   *             constructor.
   *
   * @param[in]  duration  The desired sleep duration
   */
  inline void sleep(chrono_granularity duration) {
    desired_ += duration;

    /* Provide a lower limit for the desired sleep duration passed to the
     * `sleep_for` functions. */
    auto sleep_time = std::max((duration * minimum_percentage_ / 100),
                               duration - offset_ - clock_overhead_);

    SPDLOG_LOGGER_TRACE(logger_, "[TimeKeeper] will sleep for {}.",
                        sleep_time.count());

    /* Call the respective sleep function based on the chosen threading model.
     */
    sleep_function_(sleep_time);
  }

  /**
   * @brief      Invokes a callable object repeatedly with a constant period,
   *             until some predicate is satisfied.
   *
   * @param[in]  period     The period at which the function is called
   * @param[in]  predicate  A function returning true if continuing, false
   *                        otherwise
   * @param[in]  callable   A callable object
   * @param[in]  args       Arguments to the callable object, optional
   *                        (`sizeof...(args)` can be zero)
   *
   * @tparam     Callable   A callable type
   * @tparam     Args       Argument types to the callable object
   */
  template <typename Callable, typename... Args>
  void run_every(chrono_granularity period,
                 std::function<bool(void)>&& predicate, Callable&& callable,
                 Args&&... args) {
    begin();
    while (predicate()) {
      callable(args...);
      sleep(period);
      update_offset();
    }
  };

  /**
   * @brief      Dumps the logged actual, desired, and offset durations.
   *
   * @return     A vector of the durations arranged in a fixed-size array.
   */
  std::vector<std::array<reporting_granularity, 3>> dump_log() {
    offsets_log_.shrink_to_fit();
    actual_log_.shrink_to_fit();
    desired_log_.shrink_to_fit();

    auto size = offsets_log_.size();

    std::vector<reporting_granularity> offsets_report(size);
    std::vector<reporting_granularity> actual_report(size);
    std::vector<reporting_granularity> desired_report(size);

    /* Cast the offset durations into the chosen reporting time granularity. */
    std::transform(
        offsets_log_.begin(), offsets_log_.end(), offsets_report.begin(),
        [](auto log_item) {
          return std::chrono::duration_cast<reporting_granularity>(log_item);
        });

    /* Actual and desired values are of type time_point, `time_since_epoch()`
     * 'converts' them into durations.
     *
     * Durations are casted into the chosen reporting time granularity. */
    std::transform(actual_log_.begin(), actual_log_.end(),
                   actual_report.begin(), [](auto log_item) {
                     return std::chrono::duration_cast<reporting_granularity>(
                         log_item.time_since_epoch());
                   });
    std::transform(desired_log_.begin(), desired_log_.end(),
                   desired_report.begin(), [](auto log_item) {
                     return std::chrono::duration_cast<reporting_granularity>(
                         log_item.time_since_epoch());
                   });

    std::vector<std::array<reporting_granularity, 3>> output(size);

    for (size_t i{0}; i < size; ++i) {
      output.at(i) = {actual_report.at(i), desired_report.at(i),
                      offsets_report.at(i)};
    }

    return std::move(output);
  };

  /**
   * @brief      Calculates the mean and standard deviation of calculated
   *             offsets.
   *
   * @return     A string with formatted values using the chosen reporting
   *             granularity.
   */
  auto offset_statistics() {
    if constexpr (EnableLogging) {
      offsets_log_.shrink_to_fit();

      auto [offset_mean, offset_std_dev] =
          exot::utilities::details::chrono_statistics<chrono_granularity,
                                                      reporting_granularity>(
              offsets_log_);

      return fmt::format("µ={0:.2f}{2}, σ={1:.2f}{3}",
                         static_cast<double>(offset_mean.count()),
                         static_cast<double>(offset_std_dev.count()),
                         exot::utilities::duration_unit(offset_mean),
                         exot::utilities::duration_unit(offset_std_dev));
    } else {
      return "";
    }
  };

  /**
   * @brief      Calculates the mean and deviation of differences between actual
   *             and desired sleep durations, essentially an alias of
   *             `offset_statistics()`.
   *
   * @return     A string with formatted values using the chosen reporting
   *             granularity.
   */
  auto differential_statistics() {
    if constexpr (EnableLogging) {
      actual_log_.shrink_to_fit();
      desired_log_.shrink_to_fit();

      std::vector<chrono_granularity> differences(actual_log_.size());
      std::transform(actual_log_.begin(), actual_log_.end(),
                     desired_log_.begin(), differences.begin(),
                     [](const auto& actual, const auto& desired) {
                       return (actual - desired);
                     });

      auto [diff_mean, diff_std_dev] =
          exot::utilities::details::chrono_statistics<chrono_granularity,
                                                      reporting_granularity>(
              differences);

      return fmt::format("µ={0:.2f}{2}, σ={1:.2f}{3}",
                         static_cast<double>(diff_mean.count()),
                         static_cast<double>(diff_std_dev.count()),
                         exot::utilities::duration_unit(diff_mean),
                         exot::utilities::duration_unit(diff_std_dev));
    } else {
      return "";
    }
  };

  /**
   * @brief      Calculates the mean and standard deviation of the intervals
   *             between the consecutive actual time points.
   *
   * @return     A string with formatted values using the chosen reporting
   *             granularity.
   */
  auto interval_statistics() {
    if constexpr (EnableLogging) {
      actual_log_.shrink_to_fit();

      std::vector<chrono_granularity> intervals(actual_log_.size() - 1);

      for (size_t i{0}; i < intervals.size(); ++i) {
        intervals.at(i) = std::chrono::duration_cast<chrono_granularity>(
            actual_log_.at(i + 1) - actual_log_.at(i));
      }

      auto [diff_mean, diff_std_dev] =
          exot::utilities::details::chrono_statistics<chrono_granularity,
                                                      reporting_granularity>(
              intervals);

      return fmt::format("µ={0:.2f}{2}, σ={1:.2f}{3}",
                         static_cast<double>(diff_mean.count()),
                         static_cast<double>(diff_std_dev.count()),
                         exot::utilities::duration_unit(diff_mean),
                         exot::utilities::duration_unit(diff_std_dev));
    } else {
      return "";
    }
  };

 private:
  typename clock_type::time_point desired_;
  typename clock_type::time_point now_;

  typename chrono_granularity::rep minimum_percentage_;

  chrono_granularity offset_{0};
  chrono_granularity clock_overhead_;

  std::vector<chrono_granularity> offsets_log_;

  std::vector<typename clock_type::time_point> actual_log_;
  std::vector<typename clock_type::time_point> desired_log_;

  std::shared_ptr<spdlog::logger> logger_;

  sleep_func_type sleep_function_;

  /**
   * @brief      Bootstraps the time keeper with overhead calculation and log
   *             reservation
   *
   * @param[in]  overhead_iterations  The number of iterations for overhead
   *                                  calculation
   * @param[in]  log_size_to_reserve  The log size to reserve
   */
  void bootstrap(unsigned int overhead_iterations,
                 unsigned int log_size_to_reserve) {
    auto ovh = exot::utilities::details::get_overhead<
        clock_type, exot::utilities::details::timing_overhead>();
    clock_overhead_ = ovh(overhead_iterations);

    if constexpr (EnableLogging) {
      offsets_log_.reserve(log_size_to_reserve);
      actual_log_.reserve(log_size_to_reserve);
      desired_log_.reserve(log_size_to_reserve);
    }
  };
};  // namespace exot::utilities

#if defined(__unix__) || (defined(__APPLE__) && defined(__MACH__))

/**
 * @brief      Wrapper for POSIX nanosleep that uses chrono duration
 *
 * @param[in]  duration  The sleep duration
 *
 * @tparam     Rep       Underlying type used to represent the chrono duration
 * @tparam     Period    The ratio of the chrono duration
 *
 * @return     Same return value as POSIX nanosleep
 */
template <typename Rep, typename Period>
static inline int nanosleep(
    const std::chrono::duration<Rep, Period>& duration) {
  using std::chrono::nanoseconds;
  using std::chrono::seconds;

  struct timespec sleep {
    std::chrono::duration_cast<seconds>(duration).count(),
        std::chrono::duration_cast<nanoseconds>(duration % seconds{1}).count()
  };

  return ::nanosleep(&sleep, nullptr);
}

#endif

/**
 * @brief      Busy sleep function, continuously checking if time has elapsed
 *
 * @param[in]  duration  The sleep duration
 *
 * @tparam     Rep       Underlying type used to represent the chrono duration
 * @tparam     Period    The ratio of the chrono duration
 */
template <typename Rep, typename Period, bool Yield = true>
static inline void busysleep(
    const std::chrono::duration<Rep, Period>& duration) {
  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() < start + duration) {
    if constexpr (Yield) std::this_thread::yield();
  }
}

/**
 * @brief      Gets the UTC time as a C time struct
 *
 * @return     The UTC time.
 */
static std::tm get_utc_time() {
  using clock = std::chrono::system_clock;
  auto time   = clock::to_time_t(clock::now());
  return *std::gmtime(&time);
}

/**
 * @brief      Gets the local time as a C time struct
 *
 * @return     The local time.
 */
static std::tm get_local_time() {
  using clock = std::chrono::system_clock;
  auto time   = clock::to_time_t(clock::now());
  return *std::localtime(&time);
}

}  // namespace exot::utilities
