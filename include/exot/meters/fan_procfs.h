/**
 * @file meters/fan_procfs.h
 * @author     Bruno Klopott
 * @brief      Fan speed metering module utilising values reported in procfs.
 */

#pragma once

#if defined(__linux__)

#include <fstream>
#include <string>
#include <vector>

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <spdlog/sinks/stdout_color_sinks.h>

#include <exot/meters/base.h>
#include <exot/utilities/configuration.h>
#include <exot/utilities/filesystem.h>
#include <exot/utilities/formatting.h>

namespace exot::modules {

struct fan_procfs : module {
  using return_type = unsigned;

  struct settings : public exot::utilities::configurable<settings> {
    const char* name() const { return "fan_procfs"; }

    void configure() {}
  };

  explicit fan_procfs(settings& conf) {
    if (!exot::utilities::is_readable(filepath_))
      throw std::logic_error("Fan readout is not readable.");

    string_buffer_.reserve(50);
  }

  return_type measure() {
    std::ifstream file{filepath_};
    std::getline(file, string_buffer_);
    std::getline(file, string_buffer_);

    return exot::utilities::extract_number(string_buffer_);
  }

  std::vector<std::string> header() {
    return {exot::utilities::generate_header(conf_.name(), "speed", "", "RPM")};
  }

 private:
  using logger_pointer = std::shared_ptr<spdlog::logger>;
  logger_pointer debug_log_ =
      spdlog::get("log") ? spdlog::get("log") : spdlog::stderr_color_mt("log");

  const std::string filepath_{"/proc/acpi/ibm/fan"};
  std::string string_buffer_;

  settings conf_;
};

}  // namespace exot::modules

#endif
