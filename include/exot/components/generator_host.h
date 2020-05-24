/**
 * @file components/generator_host.h
 * @author     Bruno Klopott
 * @brief      Generator host component.
 */

#pragma once

#include <chrono>
#include <mutex>
#include <set>
#include <shared_mutex>
#include <thread>
#include <tuple>
#include <type_traits>

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <spdlog/spdlog.h>

#include <spdlog/sinks/stdout_color_sinks.h>

#include <exot/framework/all.h>            // for framework support
#include <exot/generators/base.h>          // for generators base class
#include <exot/utilities/barrier.h>        // for barriers
#include <exot/utilities/configuration.h>  // for configurable
#include <exot/utilities/ostream.h>        // for ostream overloading
#include <exot/utilities/thread.h>         // for ThreadTraits
#include <exot/utilities/timing.h>         // for TimeKeeper
#include <exot/utilities/workers.h>        // for TemplatedWorker

#ifndef GENERATOR_HOST_PERFORM_VALIDATION
#define GENERATOR_HOST_PERFORM_VALIDATION false
#endif

#ifndef GENERATOR_HOST_PROVIDE_TIMING_STATISTICS
#define GENERATOR_HOST_PROVIDE_TIMING_STATISTICS false
#endif

namespace exot::components {

/**
 * @brief      Template type alias for generator token types.
 */
template <typename Duration, typename Generator>
using generator_token_type =
    std::tuple<Duration, typename Generator::subtoken_type>;

/**
 * @brief      Timed generator host.
 *
 * @tparam     Duration   The duration type
 * @tparam     Generator  The generator class, must conform to
 */
template <typename Duration, typename Generator>
class generator_host : public Generator,
                       public exot::framework::IProcess,
                       public exot::framework::Consumer<
                           generator_token_type<Duration, Generator>> {
 public:
  struct host_options {
    static constexpr bool perform_validation{
        static_cast<bool>(GENERATOR_HOST_PERFORM_VALIDATION)};
    static constexpr bool provide_timing_statistics{
        static_cast<bool>(GENERATOR_HOST_PROVIDE_TIMING_STATISTICS)};
    static constexpr bool set_affinity{true};
  };

  /* Type aliases. */
  using node_type =
      exot::framework::Consumer<generator_token_type<Duration, Generator>>;
  using state_type     = exot::framework::State;
  using state_pointer  = std::shared_ptr<state_type>;
  using token_type     = typename node_type::interface_type::value_type;
  using subtoken_type  = typename Generator::subtoken_type;
  using clock_type     = std::chrono::steady_clock;
  using duration_type  = Duration;
  using logger_pointer = std::shared_ptr<spdlog::logger>;
  using policy_type    = exot::utilities::SchedulingPolicy;
  using barrier_type   = exot::utilities::Barrier;
  using timer_duration = std::chrono::nanoseconds;
  using timer_type     = exot::utilities::TimeKeeper<
      clock_type, host_options::provide_timing_statistics, timer_duration>;
  using worker_type =
      exot::utilities::TemplatedWorker<exot::utilities::BarrierSynchronisation,
                                       exot::utilities::SpecialisedThreads>;

  using node_type::in_;

  static_assert(exot::utilities::is_duration_v<Duration>,
                "Must be a valid chrono duration type");

  static_assert(exot::modules::is_generator_module<Generator>::value,
                "Must be a conforming generator module");

  struct settings : public exot::utilities::configurable<settings>,
                    public Generator::settings {
    using base_t = exot::utilities::configurable<settings>;

    std::set<unsigned> cores;                       //! workers' allocated cores
    policy_type worker_policy{policy_type::Other};  //! workers' sched. policy
    policy_type self_policy{policy_type::Other};    //! this node's policy
    unsigned worker_priority{90u};   //! workers' scheduling priority
    unsigned self_priority{99u};     //! this node's priority
    bool use_busy_sleep{false};      //! does the host use busy sleep?
    bool busy_sleep_yield{false};    //! busy sleep yields calling thread
    unsigned start_check_period{1};  //! how often to check if started in µs
    unsigned cpu_to_pin{std::thread::hardware_concurrency() - 1};

    static_assert(
        exot::utilities::is_configurable_v<typename Generator::settings>,
        "Generator modules must satisfy is_configurable to be used with JSON "
        "configuration.");

    /* @brief The combining settings structure needs to overload this to allow
     * initialising JSON configs in inherited classes. */
    void set_json(const nlohmann::json& root) {
      base_t::set_json(root);
      Generator::settings::set_json(root);
    }

    const char* name() const { return "generator"; }

    auto describe() {
      return base_t::describe() + Generator::settings::describe();
    }

    /* @brief The JSON configuration function */
    void configure() {
      base_t::bind_and_describe_data(
          "cores", cores, "cores to pin workers to |uint[]|, e.g. [0, 2]");
      base_t::bind_and_describe_data(
          "cpu_to_pin", cpu_to_pin,
          "generator host core pinning |uint|, e.g. 5");
      base_t::bind_and_describe_data(
          "start_check_period", start_check_period,
          "state change detection update period |uint, µs|, e.g. 100");
      base_t::bind_and_describe_data("worker_policy", worker_policy,
                                     "scheduling policy of the workers |str, "
                                     "policy_type|, e.g. \"round_robin\"");
      base_t::bind_and_describe_data("self_policy", self_policy,
                                     "scheduling policy of the host |str, "
                                     "policy_type|, e.g. \"round_robin\"");
      base_t::bind_and_describe_data(
          "worker_priority", worker_priority,
          "scheduling priority of the workers |uint|, "
          "in range [0, 99], e.g. 99");
      base_t::bind_and_describe_data(
          "self_priority", self_priority,
          "scheduling priority of the host |uint|, in range [0, 99], e.g. 99");
      base_t::bind_and_describe_data("use_busy_sleep", use_busy_sleep,
                                     "should use busy sleep loop? |bool|");
      base_t::bind_and_describe_data(
          "busy_sleep_yield", busy_sleep_yield,
          "should yield thread in busy sleep loop? |bool|");

      Generator::settings::configure();
    }
  };

  /**
   * @brief      Constructor
   *
   * @param[in]  conf  The configuration structure
   */
  explicit generator_host(settings& conf)
      : Generator(conf),
        barrier_{static_cast<unsigned int>(conf_.cores.size() + 1)},
        conf_{validate_settings(conf)},
        global_state_{exot::framework::GLOBAL_STATE->get()} {
    worker_count_ = static_cast<unsigned int>(conf_.cores.size());
    local_state_  = std::make_shared<state_type>();

    if constexpr (host_options::set_affinity) {
      if (conf_.cpu_to_pin >= std::thread::hardware_concurrency()) {
        throw std::logic_error("Supplied wrong CPU to pin the generator_host");
      }
    }

    if (conf_.use_busy_sleep && !conf_.busy_sleep_yield) {
      debug_log_->info("[generator_host] using busy sleep w/o thread yield");
      timer_ = timer_type(
          debug_log_,
          &exot::utilities::busysleep<typename timer_duration::rep,
                                      typename timer_duration::period, false>);
    } else if (conf_.use_busy_sleep && conf_.busy_sleep_yield) {
      debug_log_->info("[generator_host] using busy sleep with thread yield");
      timer_ = timer_type(
          debug_log_,
          &exot::utilities::busysleep<typename timer_duration::rep,
                                      typename timer_duration::period, true>);
    } else {
      timer_ = timer_type(debug_log_);
    }

    debug_log_->info("[generator_host] using workers on cores: {}",
                     conf_.cores);

    /* Create as many workers as cores specified in settings. */
    for (auto it = conf_.cores.begin(); it != conf_.cores.end(); ++it) {
      /* Bacause cores do not necessarily map to indeces, get the position of
       * the iterator relative to the beginning of the vector. */
      auto index = std::distance(conf_.cores.begin(), it);
      auto core  = *it;
      debug_log_->debug(
          "[generator_host] creating worker with id: {}, cpu: {}",  //
          index, core);

      worker_threads_.emplace_back(worker_type(
          local_state_->get(), {std::ref(barrier_)},
          {core, conf_.worker_policy, conf_.worker_priority},
          /* Take core and index by value, this object by reference. */
          [this, core, index] {
            /* Safely access the `worker_bits_` bitset and set the `enable`
             * flag. */

            /* Uses the comma operator to access the subtoken while holding
             * the shared lock. */
            auto decomposed = this->decompose_subtoken(
                (std::shared_lock{subtoken_mtx_}, subtoken_), core, index);

            this->generate_load(decomposed, enable_flag_, core, index);
          },
          debug_log_));
    }
  }

  ~generator_host() {
    debug_log_->debug("[generator_host] joining worker threads");

    /* Join worker threads. */
    for (auto& thread : worker_threads_) {
      if (thread.joinable()) thread.join();
    }

    debug_log_->info("[generator_host] shutting down");

    if constexpr (host_options::provide_timing_statistics) {
      /* Dump timpestamped offset characteristics {actual, desired, offset}. */
      auto dump = std::move(timer_.dump_log());

      application_log_->info(
          "{},{},{}",
          exot::utilities::generate_header(
              conf_.name(), "timestamp", "",
              exot::utilities::duration_unit(
                  typename timer_type::reporting_granularity{})),
          exot::utilities::generate_header(
              conf_.name(), "desired", "",
              exot::utilities::duration_unit(
                  typename timer_type::reporting_granularity{})),
          exot::utilities::generate_header(
              conf_.name(), "offset", "",
              exot::utilities::duration_unit(
                  typename timer_type::reporting_granularity{})));

      for (auto& row : dump) {
        if constexpr (std::is_floating_point_v<
                          typename timer_type::reporting_granularity::rep>) {
          application_log_->info("{:.2f},{:.2f},{:.2f}",  //
                                 row[0].count(), row[1].count(),
                                 row[2].count());
        } else {
          application_log_->info("{},{},{}",  //
                                 row[0].count(), row[1].count(),
                                 row[2].count());
        }
      }

      /* Log mean and deviation timing statistics */
      debug_log_->info("[{}] offset: {}", conf_.name(),
                       timer_.offset_statistics());
    }
  }

  /**
   * @brief      The main process
   */
  void process() override {
    token_type token;
    std::once_flag once;

    if constexpr (host_options::set_affinity) {
      exot::utilities::ThreadTraits::set_affinity(conf_.cpu_to_pin);
    }

    exot::utilities::ThreadTraits::set_scheduling(conf_.self_policy,
                                                  conf_.self_priority);

    debug_log_->info("[generator_host] running on {}",
                     exot::utilities::thread_info());

    const auto check_p = std::chrono::microseconds{conf_.start_check_period};

    /**
     * Starting conditions:
     * - Wait until the global state is started.
     */
    while (!global_state_->is_started()) {
      std::this_thread::sleep_for(check_p);
    }

    debug_log_->info("[generator_host] starting");

    while (!local_state_->is_stopped()) {
      in_.read(token);  // read token from the input queue
      SPDLOG_LOGGER_TRACE(debug_log_, "[generator_host] read token {}", token);

      /* Decompose the input token into duration and subtoken. */
      duration_ = std::get<duration_type>(token);
      subtoken_ = std::get<subtoken_type>(token);

      if constexpr (host_options::perform_validation) {
        if (!this->validate_subtoken(subtoken_)) {
          debug_log_->critical(
              "[generator_host] Subtoken validation failed with value: {}. "
              "Aborting.",
              subtoken_);
          debug_log_->flush();
          throw std::out_of_range(fmt::format(
              "Subtoken valudation failed with value: {}", subtoken_));
        }
      }

      /* Init timing once after the first successful read from the queue. */
      std::call_once(once, [this] { timer_.begin(); });

      enable_flag_ = true;      // Workers start processing...
      barrier_.wait();          // ... after passing the first randezvous
      timer_.sleep(duration_);  // Main thread sleeps...
      enable_flag_ = false;     // Workers finish processing.

      timer_.update_offset();

      if (global_state_->is_stopped() && !in_.is_readable()) {
        local_state_->stop();
      } else if (global_state_->is_stopped() &&
                 global_state_->is_terminated()) {
        token_type dummy;
        while (!in_.is_readable()) { in_.read(dummy); }
        local_state_->stop();
      }

      barrier_.wait();  // Second randezvous point
    }
  }

  /**
   * @brief      Check if supplied configuration conforms to sensible values
   */
  settings validate_settings(settings& conf) {
    if (conf.cores.empty()) throw std::logic_error("Cores cannot be empty.");

    auto core_valid = [](auto el) {
      return el >= 0 && el < std::thread::hardware_concurrency();
    };

    if (!std::all_of(conf.cores.begin(), conf.cores.end(), core_valid)) {
      throw std::out_of_range(
          "At least one of specified cores is out of range.");
    }

    return conf;
  }

 private:
  settings conf_;
  state_pointer local_state_;
  state_pointer global_state_;
  barrier_type barrier_;

  timer_type timer_;

  duration_type duration_;
  subtoken_type subtoken_;

  logger_pointer application_log_ =
      spdlog::get("app") ? spdlog::get("app") : spdlog::stdout_color_mt("app");
  logger_pointer debug_log_ =
      spdlog::get("log") ? spdlog::get("log") : spdlog::stderr_color_mt("log");

  std::atomic_bool enable_flag_{false};
  mutable std::shared_mutex subtoken_mtx_;
  std::vector<std::thread> worker_threads_;
  unsigned worker_count_;
};  // generator_host

}  // namespace exot::components
