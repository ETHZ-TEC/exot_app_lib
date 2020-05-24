/**
 * @file meters/fan_sysfs.h
 * @author     Bruno Klopott
 * @brief      Fan speed metering module utilising values reported in sysfs.
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

namespace exot::modules {

struct fan_sysfs : module {
  using return_type = int;

  struct settings : public exot::utilities::configurable<settings> {
    const char* name() const { return "fan_sysfs"; }

    void configure() {}
  };

  explicit fan_sysfs(settings& conf) {
    {
      std::string dir{"/sys/devices/platform/thinkpad_hwmon/hwmon/"};
      std::string regex{".*fan\\d+_input$"};
      auto _ = exot::utilities::grep_directory_r(
          dir, regex, exot::utilities::Symlinks::Follow);

      if (_.size() == 0) { throw std::logic_error("No fan readouts found."); }

      filename_ = _[0];
    }

    if (!exot::utilities::is_readable(filename_))
      throw std::logic_error("Fan readout is not readable.");
  }

  return_type measure() {
    std::ifstream fan{filename_};
    return_type value;
    fan >> value;
    return value;
  }

  std::vector<std::string> header() {
    return {exot::utilities::generate_header(conf_.name(), "speed", "", "RPM")};
  }

 private:
  using logger_pointer = std::shared_ptr<spdlog::logger>;
  logger_pointer debug_log_ =
      spdlog::get("log") ? spdlog::get("log") : spdlog::stderr_color_mt("log");

  settings conf_;
  std::string filename_;
  return_type readings_;
};  // namespace exot::modules

}  // namespace exot::modules

#endif
