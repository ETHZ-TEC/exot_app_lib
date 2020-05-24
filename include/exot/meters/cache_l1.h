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
 * @file meters/cache_l1.h
 * @author     Bruno Klopott
 * @brief      Measures time to access a buffer filling the L1 cache.
 */

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>  // for std::shared_ptr
#include <string>  // for std::string
#include <thread>  // for threads
#include <vector>  // for variable-size arrays

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <spdlog/sinks/stdout_color_sinks.h>

#include <exot/meters/base.h>
#include <exot/utilities/barrier.h>
#include <exot/utilities/cache.h>
#include <exot/utilities/configuration.h>
#include <exot/utilities/helpers.h>

namespace exot::modules {

/**
 * @brief      The empirical frequency meter
 */
struct cache_l1 : module {
  using return_type  = std::vector<std::uint64_t>;
  using barrier_type = exot::utilities::Barrier;
  using probe_type   = int;

  /**
   * @brief      Settings supported by the module
   */
  struct settings : public exot::utilities::configurable<settings> {
    std::vector<unsigned> cores;  //! vector of cores
    using cache_maps_t =
        std::vector<typename exot::utilities::CacheInfo::map_type>;
    std::optional<cache_maps_t> cache_info;

    const char* name() const { return "cache_l1"; }

    /* @brief The JSON configuration function */
    void configure() {
      bind_and_describe_data(
          "cores", cores, "cores to run workers on |uint[]|, e.g. [1, 2, 3]");
      bind_and_describe_data("cache_info", cache_info,
                             "manual cache info maps |CacheInfo[]|");
    }
  };

  /**
   * @brief      Constructs the meter module
   *
   * @param      conf  The module settings structure
   */
  explicit cache_l1(settings&);

  ~cache_l1();

  /**
   * @brief      Perform measurement
   *
   * @return     A vector with time measurements (in cycles)
   */
  return_type measure();

  /**
   * @brief      Get descriptions of return values
   * @details    The descriptions contain variable names and units
   *
   * @return     A vector with descriptions
   */
  std::vector<std::string> header();

 private:
  settings conf_;
  return_type readings_;
  std::vector<std::thread> threads_;
  barrier_type barrier_;
  std::atomic<bool> flag_{true};
  std::atomic_flag worker_flag_ = ATOMIC_FLAG_INIT;

  /**
   * Parameters as defined by [1] in Algorithm 2 (p. 8):
   *
   * - n : number of L1 ways;
   * - o : log2(L1 line size), width of the 'offset' field in the address;
   * - s : log2(L1 set count), width of the 'set' field in the address;
   * - b : size of the buffer to allocate.
   *
   * [1] C. Maurice, C. Neumann, O. Heen, and A. Francillon, “C5: Cross-Cores
   * Cache Exot Channel.,” DIMVA, vol. 9148, no. 3, pp. 46–64, 2015.
   */
  int n_;  //! number of L1 ways
  int o_;  //! offset field width
  int s_;  //! set/index field width
  int b_;  //! buffer size

  using logger_pointer = std::shared_ptr<spdlog::logger>;
  logger_pointer debug_log_ =
      spdlog::get("log") ? spdlog::get("log") : spdlog::stderr_color_mt("log");
};

}  // namespace exot::modules
