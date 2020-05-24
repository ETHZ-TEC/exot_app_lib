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
 * @file meters/utilisation_procfs.cpp
 * @author     Bruno Klopott
 * @brief      Implementation of the /proc/stat utilisation metering module.
 */

#include <exot/meters/utilisation.h>

using namespace exot::modules;

utilisation_procfs::utilisation_procfs(settings& conf) : conf_{conf} {
  exot::utilities::bootstrap_cores(conf_.cores);

  debug_log_->info("[utilisation_procfs] using cores: {}", conf_.cores);
  if (conf_.raw) {
    debug_log_->info("[utilisation_procfs] reading raw /proc/stat values");
  }

  if (conf_.states.empty()) {
    conf_.states = {utilisation_procfs::cpu_state::user,
                    utilisation_procfs::cpu_state::idle};
  }

  /**
   * @brief      Lambda to check if a state is contained in the
   *             `conf_.states` vector.
   * @details    The function will also log the state, if found.
   *
   * @param[in]  state  The state
   *
   * @return     True if contained, false otherwise.
   */
  auto contains = [this](utilisation_procfs::cpu_state state) -> bool {
    auto state_to_string =
        [](utilisation_procfs::cpu_state state) -> std::string {
      if (state == utilisation_procfs::cpu_state::user)
        return "user";
      else if (state == utilisation_procfs::cpu_state::nice)
        return "nice";
      else if (state == utilisation_procfs::cpu_state::system)
        return "system";
      else
        return "idle";
    };

    if (std::find(conf_.states.begin(), conf_.states.end(), state) !=
        conf_.states.end()) {
      debug_log_->debug("[utilisation_procfs] reading state: {}",
                        state_to_string(state));
      return true;
    } else {
      return false;
    }
  };

  /* Set enable_flags_ to determine which state is to be accessed. */
  enable_flags_[0] = contains(utilisation_procfs::cpu_state::user);
  enable_flags_[1] = contains(utilisation_procfs::cpu_state::nice);
  enable_flags_[2] = contains(utilisation_procfs::cpu_state::system);
  enable_flags_[3] = contains(utilisation_procfs::cpu_state::idle);

  auto state_count   = conf_.states.size();
  auto core_count    = conf_.cores.size();
  auto readings_size = core_count * state_count;

  debug_log_->debug(
      "[utilisation_procfs] reading {} states from {} cores, {} readings in "
      "total",
      state_count, core_count, readings_size);

  previous_readings_.resize(readings_size);
  current_readings_.resize(readings_size);
  output_.resize(readings_size);

  if (std::ifstream ifs{filepath_}; !ifs.is_open()) {
    throw std::logic_error("/proc/stat is not readable");
  }

  /* The _SC_CLK_TCK sysconf value contains the number of ticks per second used
   * in utilisation reporting. */
  ticks_per_second_ = ::sysconf(_SC_CLK_TCK);
  debug_log_->trace("[utilisation_procfs] ticks per second: {}",
                    ticks_per_second_);

  /* First reading. */
  previous_time_ = std::chrono::steady_clock::now();
  once(previous_readings_);
  std::this_thread::sleep_for(std::chrono::milliseconds{100});
}

typename utilisation_procfs::return_type utilisation_procfs::measure() {
  auto current_time = std::chrono::steady_clock::now();
  once(current_readings_);

  /* If using the *processed* setting, take the current and previous readings
   * and elapsed time to obtain a relative utilisation reading, usually in the
   * range [0, 1]. Otherwise simply log the raw readings from procfs access. */
  if (!conf_.raw) {
    using float_second = std::chrono::duration<double, std::ratio<1>>;
    auto delta_time =
        std::chrono::duration_cast<float_second>(current_time - previous_time_);
    previous_time_ = current_time;

    std::transform(current_readings_.begin(), current_readings_.end(),
                   previous_readings_.begin(), output_.begin(),
                   [&](auto current, auto previous) {
                     return ((static_cast<double>(current - previous) /
                              static_cast<double>(ticks_per_second_)) /
                             delta_time.count());
                   });
    previous_readings_ = current_readings_;
  } else {
    for (size_t idx{0}; idx < output_.size(); ++idx) {
      output_.at(idx) = static_cast<double>(current_readings_.at(idx));
    }
  }

  return output_;
}

void utilisation_procfs::once(
    typename utilisation_procfs::reading_type& readings) {
  static std::string regex_string{
      "cpu(\\d+)\\s+(\\d+)\\s(\\d+)\\s(\\d+)\\s(\\d+)"};
  static std::regex regex(regex_string, std::regex_constants::ECMAScript);

  std::smatch regex_matches;

  std::ifstream file{filepath_};
  std::string string_buffer{std::istreambuf_iterator<char>(file), {}};

  auto add_readings = [&](unsigned core, size_t& idx) {
    if (core == std::stoi(regex_matches[1].str())) {
      if (enable_flags_[0]) { readings[idx++] = stoi(regex_matches[2].str()); }
      if (enable_flags_[1]) { readings[idx++] = stoi(regex_matches[3].str()); }
      if (enable_flags_[2]) { readings[idx++] = stoi(regex_matches[4].str()); }
      if (enable_flags_[3]) { readings[idx++] = stoi(regex_matches[5].str()); }
    }
  };

  size_t idx{0};
  while (std::regex_search(string_buffer, regex_matches, regex)) {
    SPDLOG_LOGGER_TRACE(debug_log_, "[utilisation_procfs] line: {}",
                        regex_matches[0].str());
    SPDLOG_LOGGER_TRACE(debug_log_, "[utilisation_procfs] core: {}",
                        regex_matches[1].str());
    SPDLOG_LOGGER_TRACE(debug_log_,
                        "[utilisation_procfs] total matches: {}, values: {}",
                        regex_matches.size(), regex_matches.size() - 2);

    for (auto core : conf_.cores) { add_readings(core, idx); }

    string_buffer = regex_matches.suffix();
  }
}

std::vector<std::string> utilisation_procfs::header() {
  std::vector<std::string> description;

  for (auto core : conf_.cores) {
    if (enable_flags_[0])
      description.push_back(
          exot::utilities::generate_header(conf_.name(), "user", core, ""));
    if (enable_flags_[1])
      description.push_back(
          exot::utilities::generate_header(conf_.name(), "nice", core, ""));
    if (enable_flags_[2])
      description.push_back(
          exot::utilities::generate_header(conf_.name(), "system", core, ""));
    if (enable_flags_[3])
      description.push_back(
          exot::utilities::generate_header(conf_.name(), "idle", core, ""));
  }

  return description;
}
