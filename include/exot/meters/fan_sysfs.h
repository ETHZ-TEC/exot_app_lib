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
