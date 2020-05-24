/**
 * @file meters/frequency_sysfs.h
 * @author     Bruno Klopott
 * @brief      Frequency meter logging values provided in sysfs' cpufreq
 *             endpoints.
 */

#pragma once

#if defined(__linux__)

#include <memory>  // for std::shared_ptr
#include <string>  // for std::string
#include <vector>  // for variable-size arrays

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <spdlog/sinks/stdout_color_sinks.h>

#include <exot/meters/base.h>
#include <exot/utilities/configuration.h>  // for configurable

namespace exot::modules {

struct frequency_sysfs : module {
  using return_type = std::vector<unsigned int>;

  /**
   * @brief      Settings supported by the module
   */
  struct settings : public exot::utilities::configurable<settings> {
    bool strict{true};
    std::vector<unsigned> cores;  //! vector of cores
    std::string source{
        "scaling_cur_freq"};  //! use scaling_* or cpuinfo_cur_freq

    const char* name() const { return "frequency_sysfs"; }

    /* @brief The JSON configuration function */
    void configure() {
      bind_and_describe_data(
          "cores", cores,
          "cores to measure frequency of |uint[]|, e.g. [0, 1, 2, 3]");
      bind_and_describe_data(
          "source", source,
          "the sysfs frequency source |str|, \"scaling_cur_freq\" "
          "(default) or \"cpuinfo_cur_freq\"");
      bind_and_describe_data(
          "strict", strict,
          "fail if cores are unavailable? |bool|, default: true");
    }
  };

  /**
   * @brief      Constructs the meter module
   *
   * @param      conf  The module settings structure
   */
  explicit frequency_sysfs(settings& conf);

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

 private:
  settings conf_;
  std::vector<std::string> filenames_;
  return_type readings_;

  using logger_pointer = std::shared_ptr<spdlog::logger>;
  logger_pointer debug_log_ =
      spdlog::get("log") ? spdlog::get("log") : spdlog::stderr_color_mt("log");
};

}  // namespace exot::modules

#endif
