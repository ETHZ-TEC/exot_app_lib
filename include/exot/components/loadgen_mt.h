/**
 * @file components/loadgen_mt.h
 * @author     Bruno Klopott
 * @brief      Basic multi-threaded load generator node.
 */

#pragma once

#include <algorithm>    // for std::distance
#include <atomic>       // for atomic types and operations
#include <bitset>       // for bitsets
#include <chrono>       // for durations
#include <cstdlib>      // for std::rand
#include <mutex>        // for call_once
#include <set>          // for std::set
#include <stdexcept>    // for throwing
#include <string>       // for std::string
#include <thread>       // for threads
#include <tuple>        // for heterogeneous containers
#include <type_traits>  // for is_floating_point_v
#include <vector>       // for variable-size arrays

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <spdlog/sinks/stdout_color_sinks.h>

#include <exot/framework/all.h>            // for framework support
#include <exot/utilities/barrier.h>        // for barriers
#include <exot/utilities/configuration.h>  // for configurable
#include <exot/utilities/ostream.h>        // for ostream overloading
#include <exot/utilities/thread.h>         // for ThreadTraits
#include <exot/utilities/timing.h>         // for TimeKeeper
#include <exot/utilities/workers.h>        // for TemplatedWorker

namespace exot::components {

using loadgen_token_type = std::tuple<std::chrono::microseconds, unsigned int>;

/**
 * @brief      Multithreaded loadgen class
 * @details    The loadgen accepts tokens for state-driven execution. The first
 *             element of the token holds a duration, for which the state
 *             indicated by the second element is to be held.
 *
 *             The second parameter is interpreted as a bitset, where individual
 *             bits indicate which worker is to be active. For example, by
 *             sending a token <5, 100ms>, workers with indexes 0 and 2 will
 *             become active (0b101, LSB order).
 */
class loadgen_mt : public exot::framework::IProcess,
                   public exot::framework::Consumer<loadgen_token_type> {
 public:
  using node_type      = exot::framework::Consumer<loadgen_token_type>;
  using state_type     = exot::framework::State;
  using state_pointer  = std::shared_ptr<state_type>;
  using token_type     = typename node_type::interface_type::value_type;
  using clock_type     = std::chrono::steady_clock;
  using logger_pointer = std::shared_ptr<spdlog::logger>;
  using policy_type    = exot::utilities::SchedulingPolicy;
  using barrier_type   = exot::utilities::Barrier;
  using worker_type =
      exot::utilities::TemplatedWorker<exot::utilities::BarrierSynchronisation,
                                       exot::utilities::SpecialisedThreads>;

  /**
   * @brief      The settings structure
   */
  struct settings : public exot::utilities::configurable<settings> {
    std::set<unsigned int> cores;            //! workers' allocated cores
    policy_type policy{policy_type::Other};  //! workers' scheduling policy
    unsigned int worker_priority{90};        //! workers' scheduling priority
    unsigned int self_priority{99};          //! this node's priority

    const char* name() const { return "generator"; }

    /* @brief The JSON configuration function */
    void configure() {
      bind_and_describe_data("cores", cores,
                             "cores to pin workers to |uint[]|, e.g. [0, 2]");
      bind_and_describe_data(
          "policy", policy,
          "scheduling policy of the workers |str, policy_type|, "
          "e.g. \"round_robin\"");
      bind_and_describe_data("worker_priority", worker_priority,
                             "scheduling priority of the workers |uint|, "
                             "in range [0, 99], e.g. 99");
      bind_and_describe_data("self_priority", self_priority,
                             "scheduling priority of the master thread |uint|, "
                             "in range [0, 99], e.g. 99");
    }
  };

  /**
   * @brief      Constructor
   *
   * @param[in]  conf  The configuration structure
   */
  explicit loadgen_mt(settings& conf)
      : name_{"generator"},
        policy_{conf_.policy},
        barrier_{static_cast<unsigned int>(conf_.cores.size() + 1)},
        conf_{validate_settings(conf)},
        global_state_{exot::framework::GLOBAL_STATE->get()} {
    worker_count_ = static_cast<unsigned int>(conf_.cores.size());
    local_state_  = std::make_shared<state_type>();

    /**
     * Helper lambda for converting `policy_type` to string.
     */
    auto policy_to_string = [](policy_type policy) -> std::string {
      if (policy == policy_type::Other) return "other";
      if (policy == policy_type::Fifo) return "fifo";
      if (policy == policy_type::RoundRobin) return "round robin";
      return "";
    };
    debug_log_->info("[{}] using scheduling policy: {}", name_,
                     policy_to_string(conf_.policy));
    debug_log_->info("[{}] using workers on cores: {}", name_, conf_.cores);

    /* Create as many workers as cores specified in settings. */
    for (auto it = conf_.cores.begin(); it != conf_.cores.end(); ++it) {
      /* Bacause cores do not necessarily map to indeces, get the position of
       * the iterator relative to the beginning of the vector. */
      auto index = std::distance(conf_.cores.begin(), it);
      auto core  = *it;
      debug_log_->debug("[{}] creating worker with id: {}, cpu: {}",  //
                        name_, index, core);

      threads_.emplace_back(worker_type(
          local_state_->get(), {std::ref(barrier_)},
          {core, policy_, conf_.worker_priority},
          /* Take core and index by value, this object by reference. */
          [this, core, index] {
            /* Safely access the `worker_bits_` bitset and set the `enable`
             * flag. */
            worker_bits_flag_.test_and_set(std::memory_order_acquire);
            bool enable = worker_bits_[index];
            worker_bits_flag_.clear(std::memory_order_release);

            /* @todo The use of `rand()` mimics the behaviour of the legacy
             * framework, mostly to avoid the workload function to be optimised
             * away. */
            volatile double a = std::rand();
            volatile double b = std::rand();

            if (enable) {
              SPDLOG_LOGGER_TRACE(
                  debug_log_,
                  "[generator->TemplatedWorker] Worker on cpu {} enabled.",
                  core);
              /* Heavy floating-point work loop. */
              while (loop_flag_) {
                a *= b;
                b -= a;
              }
            }
          },
          debug_log_));
    }
  }

  /**
   * @brief      The main process
   */
  void process() override {
    token_type token;
    std::once_flag once;

    exot::utilities::ThreadTraits::set_scheduling(policy_, conf_.self_priority);

    /**
     * Starting conditions:
     * - Wait until the global state is started.
     */
    while (!global_state_->is_started()) {
      std::this_thread::sleep_for(std::chrono::milliseconds{100});
    }

    debug_log_->info("[{}] starting", name_);

    while (!local_state_->is_stopped()) {
      in_.read(token);  //! read token from the input queue
      SPDLOG_LOGGER_TRACE(debug_log_, "[{}] read token {}", name_, token);

      /* @note       Type based access to tuples works only if each type appears
       *             exactly once. */
      auto duration  = std::get<std::chrono::microseconds>(token);
      auto specifier = std::get<unsigned int>(token);

      /* Check if input is outside of usable range.
       *
       * For example, if 4 workers are used, the allowed range of values is from
       * 0b0000 to 0b1111, or 0x0 to 0xf. To get the maximum, the operation `(1
       * << worker_count_)` would produce 0b10000, or 0x10, subtracting 1 will
       * produce the desired upper limit. */
      if (!validate_input(specifier)) {
        auto mask = (1 << conf_.cores.size()) - 1;
        debug_log_->critical(
            "[generator] cores specifier, {:#b}, out of range (mask: {:#b})! "
            "Aborting.",
            specifier, mask);
        debug_log_->flush();
        throw std::out_of_range(fmt::format(
            "Supplied cores value [{:#b}] was out of range (mask: {:#b}).",
            specifier, mask));
      }

      /* Init timing once after the first successful read from the queue. */
      std::call_once(once, [this] { timer_.begin(); });

      /* Assigns the bitset using the input token's bit representation. */
      worker_bits_ = specifier;

      loop_flag_ = true;       //! Workers start processing...
      barrier_.wait();         //! ... after passing the first randezvous
      timer_.sleep(duration);  //! Main thread sleeps...
      loop_flag_ = false;      //! Workers finish processing.

      timer_.update_offset();  //! Timing offset is calculated and updated

      /* Exit conditions:
       * - If the global state is stopped, but there is still data left in the
       *   queue, continue processing until the input queue is no longer
       *   readable.
       * - If the global state is terminated, empty the input queue and stop
       *   local execution.
       */
      if (global_state_->is_stopped() && !in_.is_readable()) {
        local_state_->stop();
      } else if (global_state_->is_stopped() &&
                 global_state_->is_terminated()) {
        token_type dummy;
        while (!in_.is_readable()) { in_.read(dummy); }
        local_state_->stop();
      }

      barrier_.wait();  //! Second randezvous point
    }
  }

  /**
   * @brief      Destructor
   */
  ~loadgen_mt() {
    /* Join worker threads. */
    for (auto& thread : threads_) {
      if (thread.joinable()) thread.join();
    }

    auto dump = std::move(timer_.dump_log());

    /** Dump timpestamped offset characteristics {actual, desired, offset}. */
    for (auto& row : dump) {
      if constexpr (std::is_floating_point_v<decltype(
                        timer_)::reporting_granularity::rep>) {
        application_log_->info("{:.2f},{:.2f},{:.2f}",  //
                               row[0].count(), row[1].count(), row[2].count());
      } else {
        application_log_->info("{},{},{}",  //
                               row[0].count(), row[1].count(), row[2].count());
      }
    }

    /* Log mean and deviation timing statistics */
    debug_log_->info("[{}] offset: {}", name_, timer_.offset_statistics());
  }

 private:
  std::string name_;  //! The name of the component, set via __FUNCTION__
  settings conf_;     //! The local configuration structure

  /**
   * A local state object
   */
  state_pointer local_state_;
  state_pointer global_state_;

  /**
   * Pointers to debug and application loggers
   */
  logger_pointer debug_log_ =
      spdlog::get("log") ? spdlog::get("log") : spdlog::stderr_color_mt("log");
  logger_pointer application_log_ =
      spdlog::get("app") ? spdlog::get("app") : spdlog::stderr_color_mt("app");

  unsigned int worker_count_;         //! Number of used workers
  std::vector<std::thread> threads_;  //! Vector holding worker threads

  /**
   * Main time-keeping component
   */
  exot::utilities::TimeKeeper<clock_type> timer_{debug_log_};

  barrier_type barrier_;  //! Thread synchronisation
  policy_type policy_;    //! Thread specialisation

  /**
   * Instruct which worker is to become active through a bitset structure,
   * allowing more flexibility.
   */
  std::bitset<16> worker_bits_;
  /**
   * `std::atomic_flag` provides a lock-free approach for protecting concurrent
   * access to a single variable
   */
  std::atomic_flag worker_bits_flag_ = ATOMIC_FLAG_INIT;

  /**
   * Atomic boolean flag indicating if workers should start execution.
   */
  std::atomic_bool loop_flag_{false};

  /**
   * @brief      Checks if provided input is sensible
   *
   * @param[in]  arg   The provided input indicating which cores are to be
   *                   active
   *
   * @return     True if within sensible range, false otherwise.
   */
  bool validate_input(unsigned int arg) {
    /**
     * This operation will be performed exactly once after the control flow
     * passes through it. Afterwards, just the value will be used.
     */
    static unsigned int validator = (1 << conf_.cores.size()) - 1;

    return (arg <= validator);
  }

  /**
   * @brief      Check if supplied configuration conforms to sensible values
   */
  settings validate_settings(settings& conf) {
    /* If no cores have been selected, construct a set of cores matching the
     * number of physical cores (0, 2, 4, 6, ...). */
    if (conf.cores.empty()) {
      for (unsigned int i{0}; i < std::thread::hardware_concurrency(); ++++i) {
        conf.cores.insert(i);
      }
    }

    /**
     * @brief      Check if specified cores are in the allowed range
     */
    auto core_valid = [](auto el) {
      return el >= 0 && el < std::thread::hardware_concurrency();
    };

    if (!std::all_of(conf.cores.begin(), conf.cores.end(), core_valid)) {
      throw std::out_of_range(
          "At least one of specified cores is out of range.");
    }

    return conf;
  }
};  // namespace exot::components

}  // namespace exot::components
