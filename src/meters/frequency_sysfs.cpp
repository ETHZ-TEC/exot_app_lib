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
 * @file meters/frequency_sysfs.cpp
 * @author     Bruno Klopott
 * @brief      Implementation of the sysfs frequency metering module.
 */

#include <exot/meters/frequency_sysfs.h>

#include <algorithm>  // for sort, unique
#include <numeric>    // for iota, set_difference

#include <fmt/format.h>   // for formatting strings
#include <fmt/ostream.h>  // for ostream support

#include <exot/utilities/configuration.h>  // for bootstrap_cores
#include <exot/utilities/filesystem.h>     // for grep_directory_r
#include <exot/utilities/formatting.h>     // for extract_number
#include <exot/utilities/ostream.h>        // for ostream operator overloads

using namespace exot::modules;

frequency_sysfs::frequency_sysfs(settings& conf) : conf_{conf} {
  std::string dir{"/sys/devices/system/cpu"};  //! the directory where cpufreq
                                               //! file nodes are found

  /* Since sysfs cpufreq nodes are usually placed as symbolic directory links
   * (and refer to "/cpufreq/policy*", we only check for the trailing number
   * and source specifier, and tell `grep_directory_r` to follow symlinks. */
  std::string regex = fmt::format(".*cpufreq/.*\\d+/{}", conf_.source);

  /* If no cores were provided, perform an autodiscovery of available frequency
   * domains, by searching for cpufreq endpoints. */
  filenames_ = exot::utilities::grep_directory_r(
      dir, regex, exot::utilities::Symlinks::Follow);

  /* If none were found, fall back to the cpu/cpu* directory listing. */
  if (filenames_.size() == 0) {
    debug_log_->warn(
        "[frequency_sysfs] automatic detection of cpufreq endpoints failed, "
        "falling back to \"cpu*\" directory discovery");

    /* Get /sys/devices/system/cpu/cpu* directories */
    auto cpu_dirs =
        exot::utilities::grep_directory(dir, std::string{".*/cpu\\d+$"});
    std::sort(cpu_dirs.begin(), cpu_dirs.end());

    debug_log_->debug("[frequency_sysfs] found {} cpu* directories",
                      cpu_dirs.size());

    /* construct the full file paths */
    for (const auto& cpu_dir : cpu_dirs) {
      filenames_.push_back(fmt::format("{}/cpufreq/{}", cpu_dir, conf_.source));
    }

    /* remove paths that are not readable */
    auto removed = exot::utilities::remove_unreadable(filenames_);

    if (filenames_.empty()) {
      debug_log_->critical(
          "[frequency_sysfs] no cpufreq endpoints are available/accessible");

      throw std::logic_error(
          fmt::format("No cpufreq endpoints are available/accessible."));
    }

    for (const auto& el : removed) {
      debug_log_->trace(
          "[frequency_sysfs] did not find a cpufreq endpoint for core {}",
          exot::utilities::extract_number(el));
    }
  } else {
    debug_log_->debug("[frequency_sysfs] found {} cpufreq endpoints",
                      filenames_.size());
  }

  std::sort(filenames_.begin(), filenames_.end());

  /* Extract the core numbers from the file paths to obtain the available range
   * of cores. */
  std::vector<unsigned> available_cores;
  for (const auto& filename : filenames_) {
    debug_log_->trace("[frequency_sysfs] extracted core {} from {}",
                      exot::utilities::extract_number(filename), filename);
    available_cores.push_back(exot::utilities::extract_number(filename));
  }

  /*If cores were provided in the settings structure... */
  if (!conf_.cores.empty()) {
    /* Bootstrap the cores vector and make sure that the provided cores
     * do not exceed hardware concurrency. */
    exot::utilities::bootstrap_cores(conf_.cores);

    /* Compute the set intersection 'available_cores ∩ conf_.cores'.
     *
     * TODO: The following procedures might be suitable candidates for
     * encapsulation. */
    std::vector<unsigned> intersection;
    std::set_intersection(available_cores.begin(), available_cores.end(),
                          conf_.cores.begin(), conf_.cores.end(),
                          std::back_inserter(intersection));

    /* If conf_.cores and the intersection are not the same, then some of the
     * supplied values are outside of the avaliable range.
     */
    if (conf_.cores != intersection) {
      /* Compute 'conf_.cores \ available_cores' to get which cores lie outside
       * of the accessible range of cpufreq endpoints.
       */
      std::vector<unsigned> difference;
      std::set_difference(conf_.cores.begin(), conf_.cores.end(),
                          available_cores.begin(), available_cores.end(),
                          std::back_inserter(difference));

      debug_log_->critical(
          "[frequency_sysfs] supplied cores {} are not available", difference);

      if (conf_.strict) {
        throw std::logic_error(
            fmt::format("Supplied cores {} are not available", difference));
      }

      /* Limit the cores to the accessible range. */
      conf_.cores = intersection;
    }

    /* If the set of cores is a proper subset of the available cores, remove the
     * positions which are not necessary. */
    if (conf_.cores != available_cores) {
      filenames_.erase(
          std::remove_if(filenames_.begin(), filenames_.end(),
                         [&](const auto& el) -> bool {
                           unsigned core{exot::utilities::extract_number(el)};
                           return std::find(intersection.begin(),
                                            intersection.end(),
                                            core) == intersection.end();
                         }),
          filenames_.end());
    }

  } else {
    /* ...otherwise set the cores to the available range. */
    conf_.cores = available_cores;
  }

  debug_log_->info("[frequency_sysfs] using source: {}, cores: {}",
                   conf_.source, conf_.cores);

  debug_log_->debug("[frequency_sysfs] using endpoints: {}", filenames_);

  readings_.resize(filenames_.size());

  /* Make sure that all specified files are readable. */
  if (!exot::utilities::all_readable(filenames_)) {
    debug_log_->critical(
        "[frequency_sysfs] at least one of cpufreq's are not readable.");

    throw std::logic_error(
        fmt::format("At least one of cpufreq's in not readable."));
  }
}

typename frequency_sysfs::return_type frequency_sysfs::measure() {
  for (decltype(filenames_.size()) i = 0; i < filenames_.size(); ++i) {
    std::ifstream cpufreq(filenames_.at(i));
    cpufreq >> readings_.at(i);
    cpufreq.close();
  }

  return readings_;
}

std::vector<std::string> frequency_sysfs::header() {
  std::vector<std::string> descriptions;
  for (auto core : conf_.cores) {
    descriptions.push_back(exot::utilities::generate_header(
        conf_.name(), conf_.source, core, DefaultUnits::frequency));
  }
  return descriptions;
}
