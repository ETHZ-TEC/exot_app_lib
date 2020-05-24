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
