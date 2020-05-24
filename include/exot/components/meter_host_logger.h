/**
 * @file components/meter_host_logger.h
 * @author     Bruno Klopott
 * @brief      Meter hosting node immediately logging measurements.
 */

#pragma once

#include <chrono>
#include <ctime>
#include <string>
#include <thread>
#include <type_traits>

#include <fmt/format.h>   // for string formatting
#include <fmt/ostream.h>  // for ostream support
#include <spdlog/spdlog.h>

#include <spdlog/sinks/stdout_color_sinks.h>

#include <exot/framework/all.h>            // for framework support
#include <exot/meters/base.h>              // for meter type trait
#include <exot/utilities/configuration.h>  // for configurable
#include <exot/utilities/formatting.h>     // for duration_{unit, to_string}
#include <exot/utilities/ostream.h>        // for ostream support
#include <exot/utilities/thread.h>         // for thread specialisation
#include <exot/utilities/timing.h>         // for TimeKeeper

#if !defined(__x86_64__)
#include <exot/utilities/timing_source.h>
#endif

#ifndef METER_SET_AFFINITY
#define METER_SET_AFFINITY true
#endif

#ifndef METER_USE_STATISTICS
#define METER_USE_STATISTICS false
#endif

#ifndef METER_LOG_SYSTEM_TIME
#define METER_LOG_SYSTEM_TIME false
#endif

#ifndef METER_NOW_FROM_TIMER
#define METER_NOW_FROM_TIMER true
#endif

namespace exot::components {

namespace details {
template <typename Duration, typename... Meters>
using meter_token_type = std::tuple<Duration, typename Meters::return_type...>;

struct MeterOptions {
  static constexpr bool use_statistics =
      static_cast<bool>(METER_USE_STATISTICS);
  static constexpr bool set_affinity =  //
      static_cast<bool>(METER_SET_AFFINITY);
  static constexpr bool log_system_time =
      static_cast<bool>(METER_LOG_SYSTEM_TIME);
  static constexpr bool now_from_timer =
      static_cast<bool>(METER_NOW_FROM_TIMER);
};

inline constexpr MeterOptions meter_options;

template <typename Lhs, typename Rhs>
inline constexpr bool is_same_vd =
    std::is_same_v<std::decay_t<Lhs>, std::decay_t<Rhs>>;
}  // namespace details

/**
 * @brief      Class for a meter host with integrated logging
 *
 * @tparam     Duration  The duration type used by the logger
 * @tparam     Meters    The meter modules
 */
template <typename Duration, typename... Meters>
class meter_host_logger : public Meters...,
                          public exot::framework::IProcess,
                          public exot::framework::Producer<
                              details::meter_token_type<Duration, Meters...>> {
 public:
  using node_type =
      exot::framework::Producer<details::meter_token_type<Duration, Meters...>>;
  using token_type     = typename node_type::interface_type::value_type;
  using state_type     = exot::framework::State;
  using state_pointer  = std::shared_ptr<state_type>;
  using logger_pointer = std::shared_ptr<spdlog::logger>;
  using clock_type     = std::chrono::steady_clock;
  using policy_type    = exot::utilities::SchedulingPolicy;
  using timer_type =
      exot::utilities::TimeKeeper<clock_type,
                                  details::meter_options.use_statistics,
                                  std::chrono::nanoseconds>;
  using timer_duration = typename timer_type::chrono_granularity;
  using node_type::out_;
  using configurable_type =
      exot::utilities::configurable<meter_host_logger<Duration, Meters...>>;

  /**
   * @brief      The settings structure, inherits from all module settings
   */
  struct settings : public exot::utilities::configurable<settings>,
                    Meters::settings... {
    Duration period{std::chrono::milliseconds{10}};
    unsigned cpu_to_pin{0};
    unsigned priority{90};
    policy_type policy{policy_type::Other};  //! workers' scheduling policy
    bool log_header{true};
    bool start_immediately{false};
    bool use_busy_sleep{false};
    bool busy_sleep_yield{false};
    unsigned start_check_period{1};  //! how often to check if started in µs

    using base_t = exot::utilities::configurable<settings>;

    static_assert(
        (exot::utilities::is_configurable_v<typename Meters::settings> && ...),
        "Meters must satisfy is_configurable to be used with JSON "
        "configuration.");

    /* @brief The configurable's component identifier */
    const char* name() const { return "host"; }

    /* @brief The combining settings structure needs to overload this to allow
     * initialising JSON configs in inherited classes. */
    void set_json(const nlohmann::json& root) {
      base_t::set_json(root);

      /* Fold expression over all meter module configure functions. */
      (..., Meters::settings::set_json(root));
    }

    std::string describe() {
      auto description = base_t::describe();
      (..., description.append(Meters::settings::describe()));
      return description;
    }

    /* @brief The JSON configuration function */
    void configure() {
      base_t::bind_and_describe_data("period", period,
                                     "sampling period |s|, e.g. 1.0e-3");
      base_t::bind_and_describe_data("pinning", cpu_to_pin,
                                     "meter host core pinning |uint|, e.g. 1"),
          base_t::bind_and_describe_data(
              "start_check_period", start_check_period,
              "state change detection update period |µs|, e.g. 100");
      base_t::bind_and_describe_data(
          "priority", priority,
          "scheduling priority |uint|, in range [0, 99], e.g. 99");
      base_t::bind_and_describe_data(
          "policy", policy,
          "scheduling policy |str, policy_type|, e.g. \"round_robin\"");
      base_t::bind_and_describe_data("log_header", log_header,
                                     "should log header? |bool|");
      base_t::bind_and_describe_data("start_immediately", start_immediately,
                                     "should start immediately? |bool|");
      base_t::bind_and_describe_data("use_busy_sleep", use_busy_sleep,
                                     "should use busy sleep loop? |bool|");
      base_t::bind_and_describe_data(
          "busy_sleep_yield", busy_sleep_yield,
          "should yield thread in busy sleep loop? |bool|");

      /* Fold expression over all meter module configure functions. */
      (..., Meters::settings::configure());
    }
  };

  static_assert((exot::modules::has_meter_function<Meters>::value && ...),
                "Mixins need to provide a measure() function.");

  meter_host_logger(settings& conf) : Meters(conf)..., conf_{conf} {
    debug_log_->info("[host] using period: {}",
                     exot::utilities::duration_to_string(conf_.period));

    if (conf_.use_busy_sleep && !conf_.busy_sleep_yield) {
      debug_log_->info("[host] using busy sleep without thread yield");
      timer_ = timer_type(
          debug_log_,
          &exot::utilities::busysleep<typename timer_duration::rep,
                                      typename timer_duration::period, false>);
    } else if (conf_.use_busy_sleep && conf_.busy_sleep_yield) {
      debug_log_->info("[host] using busy sleep with thread yield");
      timer_ = timer_type(
          debug_log_,
          &exot::utilities::busysleep<typename timer_duration::rep,
                                      typename timer_duration::period, true>);
    } else {
      debug_log_->info("[host] using regular sleep function");
      timer_ = timer_type(debug_log_);
    }

    if constexpr (details::meter_options.set_affinity) {
      debug_log_->info("[host] will run pinned to {}", conf_.cpu_to_pin);
    }

    logging_header_ = fmt::format("{}", header());
  }

  void process() override {
    auto until  = [this]() { return !global_state_->is_stopped(); };
    auto action = [this]() {
      if constexpr (details::meter_options.log_system_time) {
        application_log_->info("{},{:%FT%T}", measure(),
                               exot::utilities::get_utc_time());
      } else {
        application_log_->info("{}", measure());
      }
    };

    if constexpr (details::meter_options.set_affinity) {
      exot::utilities::ThreadTraits::set_affinity(conf_.cpu_to_pin);
    }

    exot::utilities::ThreadTraits::set_scheduling(conf_.policy, conf_.priority);

    if (!logging_header_.empty() && conf_.log_header) {
      application_log_->info("{}", logging_header_);
      debug_log_->info("[host] logging header: {}", logging_header_);
    }

    debug_log_->info("[host] running on {}", exot::utilities::thread_info());

#if !defined(__x86_64__)
    /* Call the timing facility for 1-time initialisation, if needed. */
    volatile auto _no_discard = exot::utilities::default_timing_facility([] {
      volatile auto _ = 0ull;
      _;
    });
#endif

    const auto check_p = std::chrono::microseconds{conf_.start_check_period};

    while (!global_state_->is_started() && !conf_.start_immediately) {
      std::this_thread::sleep_for(check_p);
    }

    debug_log_->info("[host] started logging");

    /* Run `action` till `until` returns false. */
    timer_.run_every(conf_.period, until, action);
  }

  /**
   * @brief      Perform measurement
   *
   * @return     Timestamped results of all individual meters
   */
  inline token_type measure() {
    if constexpr (details::meter_options.now_from_timer) {
      if constexpr (details::is_same_vd<
                        Duration, decltype(timer_.now().time_since_epoch())>) {
        auto timestamp = timer_.now().time_since_epoch();
        return std::make_tuple(std::move(timestamp),
                               std::move(Meters::measure())...);
      } else {
        auto timestamp = std::chrono::duration_cast<Duration>(
            timer_.now().time_since_epoch());
        return std::make_tuple(std::move(timestamp),
                               std::move(Meters::measure())...);
      }
    } else {
      if constexpr (details::is_same_vd<
                        Duration, decltype(std::chrono::steady_clock::now()
                                               .time_since_epoch())>) {
        auto timestamp = std::chrono::steady_clock::now().time_since_epoch();
        return std::make_tuple(std::move(timestamp),
                               std::move(Meters::measure())...);
      } else {
        auto timestamp = std::chrono::duration_cast<Duration>(
            std::chrono::steady_clock::now().time_since_epoch());
        return std::make_tuple(std::move(timestamp),
                               std::move(Meters::measure())...);
      }
    }
  }

  auto header() {
    if constexpr (details::meter_options.log_system_time) {
      return std::make_tuple(timestamp_header(), Meters::header()...,
                             datetime_header());
    } else {
      return std::make_tuple(timestamp_header(), Meters::header()...);
    }
  }

  ~meter_host_logger() {
    debug_log_->info("[host] finished logging on {:%c %Z}",
                     exot::utilities::get_utc_time());

    if constexpr (details::meter_options.use_statistics) {
      debug_log_->info("[host] timing offset statistics: {}",
                       timer_.offset_statistics());
      debug_log_->info("[host] timing interval statistics: {}",
                       timer_.interval_statistics());
    }

    debug_log_->flush();
  }

 private:
  state_pointer global_state_{exot::framework::GLOBAL_STATE->get()};
  logger_pointer application_log_ =
      spdlog::get("app") ? spdlog::get("app") : spdlog::stdout_color_mt("app");
  logger_pointer debug_log_ =
      spdlog::get("log") ? spdlog::get("log") : spdlog::stderr_color_mt("log");
  timer_type timer_;
  std::string logging_header_;

  settings conf_;

  std::string timestamp_header() {
    return exot::utilities::generate_header(
        conf_.name(), "timestamp", "",
        exot::utilities::duration_unit(conf_.period));
  }

  std::string datetime_header() {
    return exot::utilities::generate_header(conf_.name(), "datetime", "",
                                            "iso8601");
  }
};

}  // namespace exot::components
