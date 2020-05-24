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
 * @file meters/thermal_msr.cpp
 * @author     Bruno Klopott
 * @brief      Implementation of the MSR thermal metering module.
 */

#if defined(__linux__) && !defined(__ANDROID__) && defined(__x86_64__)

#include <exot/meters/thermal_msr.h>

#include <algorithm>  // for std::transform
#include <fstream>    // for ifstream
#include <thread>     // for hardware_concurrency

#include <fmt/format.h>   // for formatting strings
#include <fmt/ostream.h>  // for ostream support

#include <exot/primitives/x86_64.h>  // for logical_processors_per_core
#include <exot/utilities/bits.h>     // for bit manipulation functions
#include <exot/utilities/ostream.h>  // for ostream operator overloads

namespace exot::modules {

thermal_msr::thermal_msr(settings& conf)
    : msr_{conf_.cores}, conf_{validate_settings(conf)} {
  /* Acquire the reference temperature target.
   *
   * Bits 23:16 hold the temperature target of the particular processor
   * (TJunction). */
  temperature_target_ = exot::utilities::extract_bit_range(
      msr_.read(conf_.cores[0], IA32_TEMPERATURE_TARGET), 16u, 23u);

  /* Check if the processor supports package-level thermal management. */
  if (conf_.use_package) {
    auto cpuid_06h_eax = exot::primitives::cpuid(0x06)[0];
    if (!exot::utilities::test_bit(cpuid_06h_eax, 6u)) {
      debug_log_->warn(
          "Package temperature was requested but the processor does not "
          "support package level thermal management. Falling back to "
          "cores-only.");

      conf_.use_package = false;
    }
  }

  readings_.resize(conf_.cores.size() + (conf_.use_package ? 1 : 0));

  debug_log_->info("[thermal_msr] using cores: {}, package? {}", conf_.cores,
                   conf_.use_package);
}

typename thermal_msr::return_type thermal_msr::measure() {
  static size_t count = conf_.cores.size();

  for (unsigned i{0}; i < count; ++i) {
    readings_.at(i) = msr_.read(conf_.cores.at(i), IA32_THERM_STATUS);
  }

  if (conf_.use_package)
    readings_.at(count) = msr_.read_any(IA32_PACKAGE_THERM_STATUS);

  return get_temperatures(std::move(readings_));
}

std::vector<std::string> thermal_msr::header() {
  std::vector<std::string> description;

  for (auto core : conf_.cores) {
    description.push_back(exot::utilities::generate_header(
        conf_.name(), "core", core, DefaultUnits::temperature));
  }

  if (conf_.use_package) {
    description.push_back(exot::utilities::generate_header(
        conf_.name(), "package", "", DefaultUnits::temperature));
  }

  return description;
}

thermal_msr::settings thermal_msr::validate_settings(settings& conf) {
  /* If no cores were provided... */
  if (conf.cores.empty()) {
    for (unsigned i = 0; i < std::thread::hardware_concurrency();
         i += exot::primitives::logical_processors_per_core()) {
      conf.cores.push_back(i);
    }
  }

  return conf;
}

unsigned short thermal_msr::get_temperature(std::uint64_t raw) {
  return static_cast<unsigned short>(
      temperature_target_ - exot::utilities::extract_bit_range(raw, 16u, 22u));
}

typename thermal_msr::return_type thermal_msr::get_temperatures(
    std::vector<std::uint64_t>&& raw) {
  return_type out(raw.size());
  for (decltype(raw.size()) i = 0; i < raw.size(); ++i) {
    out.at(i) = get_temperature(raw.at(i));
  }

  return out;
}

}  // namespace exot::modules

#endif
