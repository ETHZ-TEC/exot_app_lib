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
 * @file meters/utilisation_procfs.h
 * @author     Bruno Klopott
 * @brief      Metering module for obtaining utilisation values from /proc/stat.
 */

#pragma once

#if defined(__linux__)

#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <fstream>
#include <iterator>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <spdlog/spdlog.h>

#include <spdlog/sinks/stdout_color_sinks.h>

#include <exot/meters/base.h>
#include <exot/utilities/configuration.h>
#include <exot/utilities/ostream.h>

namespace exot::modules {

/**
 * @brief      Module for reading utilisation values from procfs
 */
struct utilisation_procfs : module {
  using return_type  = std::vector<double>; /*! return type of `measure()` */
  using reading_type = std::vector<int>;    /*! internal raw reading type */

  /**
   * @brief      Enumeration for types of CPU states to access
   */
  enum class cpu_state { user, nice, system, idle };

  /**
   * @brief      Structure holding the module settings.
   */
  struct settings : public exot::utilities::configurable<settings> {
    std::vector<unsigned> cores;
    bool raw{false};
    std::vector<cpu_state> states;

    const char* name() const { return "utilisation_procfs"; }

    void configure() {
      bind_and_describe_data("cores", cores,
                             "cores to read utilisation of |uint[]|");
      bind_and_describe_data(
          "raw", raw,
          "provide raw values? |bool|, if false, provide percentage values");
      bind_and_describe_data(
          "states", states,
          "list of states to read |cpu_state[]|, choose from "
          "[\"user\", \"nice\", \"system\", \"idle\"]");
    }
  };

  /**
   * @brief      Constructs the object
   *
   * @param      conf  The settings object
   */
  explicit utilisation_procfs(settings& conf);

  /**
   * @brief      Performs a single raw access
   *
   * @param      readings  The vector holding raw readings
   */
  void once(reading_type& readings);

  /**
   * @brief      The main measurement function
   *
   * @return     A vector containing raw or processed readings
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
  reading_type previous_readings_;
  reading_type current_readings_;
  return_type output_;
  std::array<bool, 4> enable_flags_; /*! Array of booleans for enabling access
                                        to certain CPU states. */
  long ticks_per_second_; /*! Values reported in procfs are in units described
                             as ticks per second in sysconf */
  const std::string filepath_{"/proc/stat"};
  std::chrono::steady_clock::time_point previous_time_;

  using logger_pointer = std::shared_ptr<spdlog::logger>;
  logger_pointer debug_log_ =
      spdlog::get("log") ? spdlog::get("log") : spdlog::stderr_color_mt("log");
};

/**
 * @brief      Overload for cpu_state enumeration.
 *
 * @param[in]  j     The json object
 * @param      d     The enum object to write data to
 */
inline void from_json(const nlohmann::json& j,
                      utilisation_procfs::cpu_state& d) {
  if (!j.is_string()) {
    throw nlohmann::json::type_error::create(
        302, fmt::format("type must be a string, but was a {}", j.type_name()));
  } else {
    auto value = j.get<nlohmann::json::string_t>();
    if (value == "user") {
      d = utilisation_procfs::cpu_state::user;
    } else if (value == "nice") {
      d = utilisation_procfs::cpu_state::nice;
    } else if (value == "system") {
      d = utilisation_procfs::cpu_state::system;
    } else if (value == "idle") {
      d = utilisation_procfs::cpu_state::idle;
    } else {
      throw nlohmann::json::other_error::create(
          501, fmt::format("provided cpu state value \"{}\" is not "
                           "featured in the enum class",
                           value));
    }
  }
}

/**
 * @brief      Overload for serialising cpu_state enumeration.
 *
 * @param[in]  j     The json object
 * @param      d     The enum object to serialise
 */
inline void to_json(nlohmann::json& j, const utilisation_procfs::cpu_state& d) {
  switch (d) {
    case utilisation_procfs::cpu_state::user:
      j = "user";
      break;
    case utilisation_procfs::cpu_state::nice:
      j = "nice";
      break;
    case utilisation_procfs::cpu_state::system:
      j = "system";
      break;
    case utilisation_procfs::cpu_state::idle:
      j = "idle";
      break;
  }
}

}  // namespace exot::modules

#endif
