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
 * @file meters/power_msr.cpp
 * @author     Bruno Klopott
 * @brief      Implementation of the MSR power metering module.
 */

#if defined(__x86_64__) && defined(__linux__) && !defined(__ANDROID__)

#include <exot/meters/power_msr.h>

namespace exot::modules {

power_msr::power_msr(settings& conf) : conf_{conf}, msr_{} {
  /* If no domains were provided, fall back to default. */
  if (conf_.domains.empty()) {
    conf_.domains = {rapl_domain::pp0, rapl_domain::pkg};
  }

  for (auto domain : conf_.domains) {
    debug_log_->info("[power_msr] using domain: {}", domain_to_string(domain));
  }

  /* Allocate output vector. */
  output_.resize(conf_.domains.size());

  /* Get energy unit in which energy is reported, defined as "Energy related
   * information (in Joules) is based on the multiplier, 1/2^ESU; where ESU is
   * an unsigned integer represented by bits 12:8." in "Intel® 64 and IA-32
   * Architectures Software Developer’s Manual", 2016. */
  energy_status_unit_ =
      1.0 / static_cast<double>(1ULL << exot::utilities::extract_bit_range(
                                    msr_.read_first(RAPL_POWER_UNIT), 8u, 12u));

  debug_log_->info("[power_msr] using RAPL power unit: {:e}",
                   energy_status_unit_);

  /* Bootstrap readings */
  previous_time_point_ = std::chrono::steady_clock::now();
  for (auto domain : conf_.domains) {
    old_readings_[domain] = get_energy(domain);
  }
  /* Sleeping for a short moment so that there is a meaningful value once the
   * meter is started. In most processors the values in the MSR appear every
   * ~1ms, if the time between consecutive reads is shorter, the power will be
   * reported as 0. */
  std::this_thread::sleep_for(std::chrono::milliseconds{5});
}

typename power_msr::return_type power_msr::measure() {
  auto now = std::chrono::steady_clock::now();
  double elapsed_time =
      std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1>>>(
          now - previous_time_point_)
          .count();  // elapsed time in seconds
  previous_time_point_ = now;

  auto i = 0; /*! used to access output elements in a range-for loop. */

  /* Iterate through configured power domains and compute average power over
   * the elapsed_time period. */
  for (auto domain : conf_.domains) {
    readings_[domain] = get_energy(domain);
    output_.at(i++) =
        energy_to_power(readings_[domain], old_readings_[domain], elapsed_time);
  }

  old_readings_ = readings_;

  return output_;
}

std::vector<std::string> power_msr::header() {
  std::vector<std::string> description;
  for (auto domain : conf_.domains) {
    description.push_back(exot::utilities::generate_header(
        conf_.name(), "rapl_domain", domain_to_string(domain),
        DefaultUnits::power));
  }
  return description;
}

std::string power_msr::domain_to_string(rapl_domain d) {
  switch (d) {
    case rapl_domain::pp0:
      return "pp0";
    case rapl_domain::pp1:
      return "pp1";
    case rapl_domain::pkg:
      return "pkg";
    case rapl_domain::sys:
      return "sys";
    default:
      return "";
  }
}

}  // namespace exot::modules

#endif
