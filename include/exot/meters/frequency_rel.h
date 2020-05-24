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
 * @file meters/frequency_rel.h
 * @author     Bruno Klopott
 * @brief      Empirical frequency meter module.
 */

#pragma once

#include <atomic>
#include <chrono>
#include <memory>  // for std::shared_ptr
#include <string>  // for std::string
#include <thread>  // for threads
#include <vector>  // for variable-size arrays

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <spdlog/sinks/stdout_color_sinks.h>

#include <exot/meters/base.h>
#include <exot/utilities/barrier.h>
#include <exot/utilities/configuration.h>  // for configurable
#include <exot/utilities/helpers.h>

namespace exot::modules {

namespace details {

/**
 * @brief      The probing function
 *
 * @tparam     T          An arithmetic type (e.g. unsigned, double)
 * @tparam     Iter       Interaction count
 * @tparam     <unnamed>  Template helper
 *
 * @return     The probe duration in nanoseconds
 */
template <typename T, size_t Iter,
          typename = std::enable_if_t<std::is_arithmetic_v<T>>>
inline std::chrono::nanoseconds probe() {
  volatile T a = static_cast<T>(1), b = static_cast<T>(2);

  auto start = std::chrono::steady_clock::now();

  for (decltype(Iter) i{0}; i < Iter; ++i) {
    a += b;
    b *= a;
  }

  exot::utilities::do_not_optimise(a, b);

  return (std::chrono::steady_clock::now() - start);
}

}  // namespace details

/**
 * @brief      The empirical frequency meter
 */
struct frequency_rel : module {
  using return_type   = std::vector<double>;
  using duration_type = std::chrono::duration<double, std::nano>;
  using barrier_type  = exot::utilities::Barrier;
  using probe_type    = int;
  static const unsigned probe_iterations{1024u};

  /**
   * @brief      Settings supported by the module
   */
  struct settings : public exot::utilities::configurable<settings> {
    std::vector<unsigned> cores;  //! vector of cores

    const char* name() const { return "frequency_rel"; }

    /* @brief The JSON configuration function */
    void configure() {
      bind_and_describe_data(
          "cores", cores,
          "cores to measure relative frequency on |uint[]|, e.g. [0, 2]");
    }
  };

  /**
   * @brief      Constructs the meter module
   *
   * @param      conf  The module settings structure
   */
  explicit frequency_rel(settings&);

  ~frequency_rel();

  /**
   * @brief      Perform measurement
   *
   * @return     A vector with frequency readings
   */
  return_type measure();

  /**
   * @brief      Get descriptions of return values
   * @details    The descriptions contain variable names and units
   *
   * @return     A vector with descriptions
   */
  std::vector<std::string> header();

  /**
   * @brief      Get the reference probe duration
   *
   * @param[in]  repetitions  The number of repeated measurements used to
   *                          provide the average probe duration
   *
   * @return     The average probe duration
   */
  static inline duration_type reference_probe_duration(unsigned repetitions);

 private:
  settings conf_;
  return_type readings_;
  std::vector<duration_type> references_;
  std::vector<std::thread> threads_;
  barrier_type barrier_;
  std::atomic<bool> flag_{false};
  std::atomic_flag worker_flag_ = ATOMIC_FLAG_INIT;

  using logger_pointer = std::shared_ptr<spdlog::logger>;
  logger_pointer debug_log_ =
      spdlog::get("log") ? spdlog::get("log") : spdlog::stderr_color_mt("log");
};

}  // namespace exot::modules
