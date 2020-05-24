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
 * @file components/generator_ffb.h
 * @author     Bruno Klopott
 * @brief      Generator host component.
 */

#pragma once

#include <algorithm>  // for std::distance
#include <atomic>     // for atomic types and operations
#include <bitset>     // for bitsets
#include <chrono>
#include <cstdlib>  // for std::rand
#include <memory>   // for std::shared_ptr
#include <mutex>
#include <set>
#include <shared_mutex>
#include <stdexcept>  // for throwing
#include <string>     // for std::string
#include <thread>
#include <tuple>
#include <type_traits>
#include <vector>  // for variable-size arrays

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <spdlog/spdlog.h>

#include <spdlog/sinks/stdout_color_sinks.h>

//#include <covert/modules/base.h>
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
class generator_ffb : public Generator,
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

    /// @defgroup gen_worker_settings Generator host's worker settings
    /// @{
    unsigned core;                  //! worker' allocated core
    std::vector<unsigned> cores;    //! workers' allocated cores
    bool should_pin_workers{true};  //! should pin the worker threads?
    policy_type worker_policy{policy_type::Other};  //! workers' sched. policy
    unsigned worker_priority{90u};  //! workers' scheduling priority
    unsigned threshold{100};        //! worker measurement threshold
    unsigned num_hits{20};   //! worker number of necessary measurement hits
    unsigned num_probes{5};  //! worker number of necessary measurement hits
    unsigned host_timeout_s{
        60};  //! worker number of necessary measurement hits
    /// @}

    /// @defgroup gen_host_settings Generator host's master thread settings
    /// @{
    unsigned host_pinning{//! cpu to pin host's master thread to
                          std::thread::hardware_concurrency() - 1};
    bool should_pin_host{true};  //! should pin the master thread?
    policy_type host_policy{policy_type::Other};  //! this node's policy
    unsigned host_priority{99u};                  //! this node's priority
    /// @}

    bool use_busy_sleep{false};      //! does the host use busy sleep?
    bool busy_sleep_yield{false};    //! busy sleep yields calling thread
    unsigned start_check_period{1};  //! how often to check if started in µs

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
          "cores", cores,
          "cores to pin worker to |uint|, e.g. [0]"
          "further cores will be ignored - this is for compatibility reasons");
      base_t::bind_and_describe_data(
          "should_pin_workers", should_pin_workers,
          "should pin the workers? |bool|, default 'true'");
      base_t::bind_and_describe_data("worker_policy", worker_policy,
                                     "scheduling policy of the workers |str, "
                                     "policy_type|, e.g. \"round_robin\"");
      base_t::bind_and_describe_data(
          "worker_priority", worker_priority,
          "scheduling priority of the workers |uint|, "
          "in range [0, 99], e.g. 99");
      base_t::bind_and_describe_data(
          "threshold", threshold,
          "measurement threshold used to detect a frequency change"
          "|uint|");
      base_t::bind_and_describe_data(
          "num_probes", num_probes,
          "number of necessary probes for one measurement"
          "|uint|");
      base_t::bind_and_describe_data(
          "num_hits", num_hits,
          "number of necessary measurement  hits to detect a frequency change"
          "|uint|");

      base_t::bind_and_describe_data(
          "host_timeout_s", host_timeout_s,
          "generator host timeout in seconds |uint|, e.g. 20");
      base_t::bind_and_describe_data(
          "host_pinning", host_pinning,
          "generator host core pinning |uint|, e.g. 5");
      base_t::bind_and_describe_data(
          "should_pin_host", should_pin_host,
          "should pin the host? |bool|, default 'true'");
      base_t::bind_and_describe_data("host_policy", host_policy,
                                     "scheduling policy of the host |str, "
                                     "policy_type|, e.g. \"round_robin\"");
      base_t::bind_and_describe_data(
          "host_priority", host_priority,
          "scheduling priority of the host |uint|, in range [0, 99], e.g. 99");

      base_t::bind_and_describe_data(
          "start_check_period", start_check_period,
          "state change detection update period |uint, µs|, e.g. 100");
      base_t::bind_and_describe_data("use_busy_sleep", use_busy_sleep,
                                     "should use busy sleep loop? |bool|");
      base_t::bind_and_describe_data(
          "busy_sleep_yield", busy_sleep_yield,
          "should yield thread in busy sleep loop? |bool|");

      Generator::settings::configure();
    }
  };

  /**
   * @brief      The probing function
   *
   * @tparam     T          An arithmetic type (e.g. unsigned, double)
   * @tparam     Iter       Interaction count
   * @tparam     <unnamed>  Template helper
   *
   * @return     The probe duration in nanoseconds
   */
  template <typename T, unsigned Iter,
            typename = std::enable_if_t<std::is_arithmetic_v<T>>>
  inline std::chrono::nanoseconds probe() {
    volatile T a = std::rand(), b = std::rand();

    auto start = std::chrono::steady_clock::now();
    for (unsigned i{0}; i < Iter; ++i) {
      a += b + static_cast<T>(i);
      b *= a;
    }
    return (std::chrono::steady_clock::now() - start);
  }

  /**
   * @brief      Constructor
   *
   * @param[in]  conf  The configuration structure
   */
  explicit generator_ffb(settings& conf)
      : Generator(conf),
        barrier_{static_cast<unsigned int>(1)},
        conf_{validate_settings(conf)},
        global_state_{exot::framework::GLOBAL_STATE->get()} {
    worker_count_ = static_cast<unsigned int>(1);
    local_state_  = std::make_shared<state_type>();

    if constexpr (host_options::set_affinity) {
      if (conf_.host_pinning >= std::thread::hardware_concurrency()) {
        throw std::logic_error("Supplied wrong CPU to pin the generator_ffb");
      }
    }

    if (conf_.use_busy_sleep && !conf_.busy_sleep_yield) {
      debug_log_->info("[generator_ffb] using busy sleep w/o thread yield");
      timer_ = timer_type(
          debug_log_,
          &exot::utilities::busysleep<typename timer_duration::rep,
                                      typename timer_duration::period, false>);
    } else if (conf_.use_busy_sleep && conf_.busy_sleep_yield) {
      debug_log_->info("[generator_ffb] using busy sleep with thread yield");
      timer_ = timer_type(
          debug_log_,
          &exot::utilities::busysleep<typename timer_duration::rep,
                                      typename timer_duration::period, true>);
    } else {
      timer_ = timer_type(debug_log_);
    }

    /* Create a worker. */
    debug_log_->debug("[generator_ffb] creating worker on core: {}",  //
                      conf_.core);

    auto core       = conf_.cores.front();
    conf_.core      = conf_.cores.front();
    auto num_probes = conf_.num_probes;
    std::chrono::nanoseconds threshold{conf_.threshold};
    auto num_hits = conf_.num_hits;

    worker_threads_.emplace_back(worker_type(
        local_state_->get(),
        // config for SynchronisationPolicy->BarrierSynchronisation
        {std::ref(barrier_)},
        // config for ThreadingPolicy->SpecialisedThreads
        {conf_.core, conf_.should_pin_workers, conf_.worker_policy,
         conf_.worker_priority},
        /* Take core and index by value, this object by reference. */
        [this, core, num_probes, threshold, num_hits] {
          unsigned int cnt_hit = 0;
          //! One utilisation cycle should be 100us long. This way sleep is not
          //! called too often but the loagen_bc is still sampling way faster
          //! (>10x) than the governor.
          std::chrono::nanoseconds old_measurement,
              new_measurement = probe<double, 1024>();
          unsigned int num_probes =
              std::chrono::microseconds(100) / new_measurement + 1;
          auto sleep_duration =
              this->get_sleep_time(subtoken_, new_measurement);
          int change_count = 0;
          //! While no timeout has occured try to change the frequency until the
          //! desired number of frequency changes have been reached
          while (enable_flag_ || subtoken_ != change_count) {
            old_measurement = new_measurement;
            timer_.sleep(sleep_duration);
            new_measurement = std::chrono::nanoseconds(0);
            for (unsigned int cnt = 0; cnt < num_probes; cnt++) {
              new_measurement += probe<double, 1024>();
            }
            new_measurement /= num_probes;
            auto difference =
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    new_measurement - old_measurement);
            //! If the difference between two measurements is bigger than the
            //! threshold, increase the hit count
            if ((difference >= difference.zero()) ? (difference > threshold)
                                                  : (-difference > threshold)) {
              cnt_hit++;
              if (cnt_hit == num_hits) {
                (difference >= difference.zero()) ? (change_count += 1)
                                                  : (change_count -= 1);
              }
            } else {
              cnt_hit = 0;
            }
          }
          enable_flag_ = false;
          worker_done_.notify_one();  // Release main threat from wait
        },
        debug_log_));
  }

  ~generator_ffb() {
    debug_log_->debug("[generator_ffb] joining worker threads");

    /* Join worker threads. */
    for (auto& thread : worker_threads_) {
      if (thread.joinable()) thread.join();
    }

    debug_log_->info("[generator_ffb] shutting down");

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
      if (conf_.should_pin_host)
        exot::utilities::ThreadTraits::set_affinity(conf_.host_pinning);
    }

    exot::utilities::ThreadTraits::set_scheduling(conf_.host_policy,
                                                  conf_.host_priority);

    debug_log_->info("[generator_ffb] running on {}",
                     exot::utilities::thread_info());

    const auto check_p = std::chrono::microseconds{conf_.start_check_period};

    /**
     * Starting conditions:
     * - Wait until the global state is started.
     */
    while (!global_state_->is_started()) {
      std::this_thread::sleep_for(check_p);
    }

    debug_log_->info("[generator_ffb] starting");

    std::chrono::seconds timeout_s(conf_.host_timeout_s);

    while (!local_state_->is_stopped()) {
      in_.read(token);  // read token from the input queue
      SPDLOG_LOGGER_TRACE(debug_log_, "[generator_ffb] read token {}", token);

      /* Decompose the input token into duration and subtoken. */
      duration_ = std::get<duration_type>(token);
      subtoken_ = std::get<subtoken_type>(token);

      if constexpr (host_options::perform_validation) {
        if (!this->validate_subtoken(subtoken_)) {
          debug_log_->critical(
              "[generator_ffb] Subtoken validation failed with value: {}. "
              "Aborting.",
              subtoken_);
          debug_log_->flush();
          throw std::out_of_range(fmt::format(
              "Subtoken valudation failed with value: {}", subtoken_));
        }
      }

      std::unique_lock<std::mutex> timeout_lock(timeout_mutex_);
      /* Init timing once after the first successful read from the queue. */
      std::call_once(once, [this] { timer_.begin(); });

      enable_flag_ = true;  // Workers start processing...
      barrier_.wait();      // ... after passing the first randezvous
      worker_done_.wait_for(
          timeout_lock,
          timeout_s);  //! TODO check if wait_for or similar function timed-out;
                       //! the return value can be accessed as type
                       //! std::cv_status e.g. with value of
                       //! std::cv_status::timeout ->  use this to replace the
                       //! enable_flag_?
      if (enable_flag_) {
        debug_log_->info("[generator_ffb] Stopping due to timeout");
        enable_flag_ = false;        // Workers finish processing.
        global_state_->terminate();  //! Abort exection due to an error...
      }

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
    auto core_valid = [](auto el) {
      return el >= 0 && el < std::thread::hardware_concurrency();
    };

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

  std::mutex timeout_mutex_;
  std::condition_variable worker_done_;

};  // generator_ffb

}  // namespace exot::components
