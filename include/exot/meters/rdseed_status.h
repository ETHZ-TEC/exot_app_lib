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
 * @file meters/rdseed_status.h
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
#include <exot/utilities/barrier.h>
#include <exot/utilities/configuration.h>
#include <exot/utilities/thread.h>

namespace exot::modules {

/**
 * @brief      Measurement module for checking if a RDSEED operation can
 *             complete immediately.
 */
struct rdseed_status : module {
  using raw_type    = std::uint8_t;
  using return_type = bool;

  /**
   * @brief      Settings supported by the module
   */
  struct settings : public exot::utilities::configurable<settings> {
    unsigned core{0u};

    const char* name() const { return "rdseed_status"; }
    void configure() {
      bind_and_describe_data("core", core, "worker pinning |uint|");
    }
  };

  /**
   * @brief      Constructs the meter module
   *
   * @param      conf  The module settings
   */
  explicit rdseed_status(settings& conf) : barrier_(2), conf_{conf} {
    worker_thread_ = std::move(std::thread([this]() {
      exot::utilities::ThreadTraits::set_affinity(conf_.core);
      exot::utilities::ThreadTraits::set_scheduling(
          exot::utilities::SchedulingPolicy::RoundRobin, 90);
      debug_log_->debug("[rdseed_status] worker thread: {}",
                        exot::utilities::thread_info());

      while (active_.load()) {
        barrier_.wait();
        exot::primitives::seed64();
        exot::primitives::seed64();
        exot::primitives::seed64();
        exot::primitives::seed64();
        exot::primitives::check_seed64(reading_);
        barrier_.wait();
      }
    }));
  }

  ~rdseed_status() {
    active_.store(false);
    barrier_.wait();
    barrier_.wait();
    if (!worker_thread_.joinable()) barrier_.force_progress();
    worker_thread_.join();
  }

  /**
   * @brief      Perform module measurement
   *
   * @return     The RDSEED operation status
   */
  return_type measure() {
    barrier_.wait();
    barrier_.wait();
    return static_cast<return_type>(reading_);
  }

  /**
   * @brief      Get descriptions of return values
   * @details    The descriptions contain variable names and units
   *
   * @return     A vector with descriptions
   */
  std::vector<std::string> header() {
    return {exot::utilities::generate_header(conf_.name(), "status", "", "")};
  }

 private:
  using logger_pointer = std::shared_ptr<spdlog::logger>;
  logger_pointer debug_log_ =
      spdlog::get("log") ? spdlog::get("log") : spdlog::stderr_color_mt("log");

  settings conf_;
  raw_type reading_;

  exot::utilities::Barrier barrier_;
  std::thread worker_thread_;
  std::atomic_bool active_{true};
};

}  // namespace exot::modules

#endif
