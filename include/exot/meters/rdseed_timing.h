/**
 * @file meters/rdseed_timing.h
 * @author     Bruno Klopott
 * @brief      Measures contention to the hardware random number generator.
 */

#pragma once

#if defined(__x86_64__)

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include <spdlog/spdlog.h>

#include <spdlog/sinks/stdout_color_sinks.h>

#include <exot/meters/base.h>
#include <exot/primitives/rng.h>
#include <exot/primitives/tsc.h>
#include <exot/utilities/barrier.h>
#include <exot/utilities/configuration.h>
#include <exot/utilities/thread.h>
#include <exot/utilities/timing.h>

namespace exot::modules {

/**
 * @brief      Measurement module for checking if a RDSEED operation can
 *             complete immediately.
 */
struct rdseed_timing : module {
  using return_type = std::uint64_t;

  /**
   * @brief      Settings supported by the module
   */
  struct settings : public exot::utilities::configurable<settings> {
    unsigned core{0u};

    const char* name() const { return "rdseed_timing"; }
    void configure() {
      bind_and_describe_data("core", core, "worker pinning |uint|");
    }
  };

  /**
   * @brief      Constructs the meter module
   *
   * @param      conf  The module settings
   */
  explicit rdseed_timing(settings& conf) : barrier_(2), conf_{conf} {
    worker_thread_ = std::move(std::thread([this]() {
      exot::utilities::ThreadTraits::set_affinity(conf_.core);
      exot::utilities::ThreadTraits::set_scheduling(
          exot::utilities::SchedulingPolicy::RoundRobin, 90);
      debug_log_->debug("[rdseed_timing] worker thread: {}",
                        exot::utilities::thread_info());

      while (active_.load()) {
        barrier_.wait();
        exot::primitives::seed64();
        exot::primitives::seed64();
        exot::primitives::seed64();
        exot::primitives::seed64();
        reading_ = exot::utilities::timeit<exot::primitives::SerialisingTSC>(
            exot::primitives::seed64_until_ok);
        barrier_.wait();
      }
    }));
  }

  ~rdseed_timing() {
    active_.store(false);
    barrier_.wait();
    barrier_.wait();
    if (!worker_thread_.joinable()) barrier_.force_progress();
    worker_thread_.join();
  }

  /**
   * @brief      Perform module measurement
   *
   * @return     The cycles required to successfuly complete an RDSEED operation
   */
  return_type measure() {
    barrier_.wait();
    barrier_.wait();
    return reading_;
  }

  /**
   * @brief      Get descriptions of return values
   * @details    The descriptions contain variable names and units
   *
   * @return     A vector with descriptions
   */
  std::vector<std::string> header() {
    return {exot::utilities::generate_header(conf_.name(), "cycles", "", "")};
  }

 private:
  using logger_pointer = std::shared_ptr<spdlog::logger>;
  logger_pointer debug_log_ =
      spdlog::get("log") ? spdlog::get("log") : spdlog::stderr_color_mt("log");

  settings conf_;
  return_type reading_;

  exot::utilities::Barrier barrier_;
  std::thread worker_thread_;
  std::atomic_bool active_{true};
};

}  // namespace exot::modules

#endif
