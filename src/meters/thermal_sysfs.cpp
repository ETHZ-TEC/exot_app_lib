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
 * @file meters/thermal_sysfs.cpp
 * @author     Bruno Klopott
 * @brief      Implementation of the sysfs thermal metering module.
 */

#if defined(__linux__)

#include <exot/meters/thermal_sysfs.h>

namespace exot::modules {

thermal_sysfs::thermal_sysfs(settings& conf) : conf_{conf} {
  std::string dir{"/sys/devices/virtual/thermal"};
  std::string regex{".*zone\\d+/temp$"};  //! regular expression that matches
                                          //! valid thermal zone access files

  filenames_ = exot::utilities::grep_directory_r(dir, regex);

  /* Since thermal zones are counted from 0, (size - 1) should provide the
   * last available zone. */
  auto max_zone = filenames_.size() - 1;

  if (!conf_.zones.empty()) {
    /* If zones have been provided via CLI...
     * Sort and remove duplicate zones. */
    std::sort(conf_.zones.begin(), conf_.zones.end());
    conf_.zones.erase(std::unique(conf_.zones.begin(), conf_.zones.end()),
                      conf_.zones.end());

    /* If any of the provided zones are not available... */
    if (std::any_of(conf_.zones.begin(), conf_.zones.end(), [=](unsigned zone) {
          auto larger = zone > max_zone;
          if (larger)
            debug_log_->warn("Thermal zone {} is not available.", zone);
          return larger;
        })) {
      /* Limit the zones vector to available zones. */
      conf_.zones.erase(
          std::find_if(conf_.zones.begin(), conf_.zones.end(),
                       [max_zone](unsigned zone) { return zone > max_zone; }),
          conf_.zones.end());
    }

    filenames_ = {};

    for (auto zone : conf_.zones) {
      filenames_.emplace_back(fmt::format(
          "/sys/devices/virtual/thermal/thermal_zone{}/temp", zone));
    }
  } else {
    /* ...otherwise, initialise zones vector to available zones. */
    conf_.zones.resize(filenames_.size());
    std::iota(conf_.zones.begin(), conf_.zones.end(), 0);
  }

  debug_log_->info("[thermal_sysfs] using zones: {}", conf_.zones);

  readings_.resize(filenames_.size());

  if (!exot::utilities::all_readable(filenames_))
    throw std::logic_error("At least one thermal zone is not readable.");
}

typename thermal_sysfs::return_type thermal_sysfs::measure() {
  for (decltype(filenames_.size()) i = 0; i < filenames_.size(); ++i) {
    std::ifstream zone(filenames_.at(i));
    unsigned int value;
    zone >> value;
    readings_.at(i) = (value > 1000) ? static_cast<float>(value) / 1000
                                     : static_cast<float>(value);
    zone.close();
  }

  return readings_;
}

std::vector<std::string> thermal_sysfs::header() {
  std::vector<std::string> description;
  for (auto zone : conf_.zones) {
    description.push_back(exot::utilities::generate_header(
        conf_.name(), "zone", zone, DefaultUnits::temperature));
  }
  return description;
}

}  // namespace exot::modules

#endif
