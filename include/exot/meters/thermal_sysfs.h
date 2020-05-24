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
 * @file meters/thermal_sysfs.h
 * @author     Bruno Klopott
 * @brief      Thermal metering module utilising values reported in sysfs.
 */

#pragma once

#if defined(__linux__)

#include <algorithm>  // for std::transform
#include <fstream>    // for ifstream
#include <string>     // for std::string
#include <thread>     // for hardware_concurrency
#include <vector>     // for variable-size arrays

#include <fmt/format.h>   // for formatting strings
#include <fmt/ostream.h>  // for ostream support
#include <spdlog/spdlog.h>

#include <spdlog/sinks/stdout_color_sinks.h>

#include <exot/meters/base.h>
#include <exot/utilities/configuration.h>  // for configurable
#include <exot/utilities/filesystem.h>     // for filesystem operations
#include <exot/utilities/ostream.h>        // for ostream operator overloads

namespace exot::modules {

/**
 * @brief      Measurement module for reading thermal information from sysfs
 */
struct thermal_sysfs : module {
  using return_type = std::vector<float>;

  /**
   * @brief      Settings supported by the module
   */
  struct settings : public exot::utilities::configurable<settings> {
    std::vector<unsigned> zones;

    const char* name() const { return "thermal_sysfs"; }

    void configure() {
      bind_and_describe_data("zones", zones,
                             "list of zones to read |uint[]|, e.g. [0, 1]");
    }
  };

  /**
   * @brief      Constructs the meter module
   *
   * @param      conf  The module settings
   */
  explicit thermal_sysfs(settings& conf);

  /**
   * @brief      Perform module measurement
   *
   * @return     A vector with thermal zone temperature readings
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
  using logger_pointer = std::shared_ptr<spdlog::logger>;
  logger_pointer debug_log_ =
      spdlog::get("log") ? spdlog::get("log") : spdlog::stderr_color_mt("log");

  settings conf_;
  std::vector<std::string> filenames_;
  return_type readings_;
};

}  // namespace exot::modules

#endif
