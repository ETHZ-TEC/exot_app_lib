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
 * @file utilities/platform_id.cpp
 * @author     Bruno Klopott
 * @brief      Implementation of the platform identifying functions from @ref
 *             platform_id.h.
 */

#include <exot/utilities/platform_id.h>

#include <algorithm>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <thread>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include <exot/utilities/bits.h>
#include <exot/utilities/filesystem.h>
#include <exot/utilities/formatting.h>
#include <exot/utilities/ostream.h>

/* Local helpers */
namespace {

/**
 * @brief      Gets the key from key-value pairs found in cpuinfo
 *
 * @param[in]  line  The line containing the key-value pair
 *
 * @return     The key.
 */
std::string get_key(const std::string& line) {
  if (!line.empty()) {
    auto split = exot::utilities::split_string(line, ':');
    if (split.size() >= 1) { return exot::utilities::trim_string(split[0]); }
  }

  return std::string{};
}

/**
 * @brief      Gets the value from key-value pairs found in cpuinfo
 *
 * @param[in]  line  The line containing the key-value pair
 *
 * @return     The value.
 */
std::string get_value(const std::string& line) {
  std::string value;

  if (!line.empty()) {
    auto split = exot::utilities::split_string(line, ':');
    /* If the split size is 1 or less, the line in /proc/cpuinfo did not contain
     * a key-value pair. */
    if (split.size() > 1) { value = exot::utilities::trim_string(split[1]); }
  }

  return value;
}

}  // namespace

namespace exot::utilities {

std::optional<std::string> get_kernel_version() {
  return get_long_string_value_from_file("/proc/version");
}

std::vector<std::string> get_cpuinfo_keys() {
  std::vector<std::string> output;
  const auto path = "/proc/cpuinfo";

  std::ifstream file(path);

  if (is_readable(path)) {
    for (std::string line; std::getline(file, line);) {
      if (auto key = get_key(line); !key.empty() && !line.empty())
        output.push_back(key);
    }

    /* Deduplicate the results. */
    std::sort(output.begin(), output.end());
    output.erase(std::unique(output.begin(), output.end()), output.end());
  }

  return output;
}

std::vector<std::map<std::string, std::string>>
get_complete_cpuinfo_as_map_array() {
  std::vector<std::map<std::string, std::string>> output;
  const auto path = "/proc/cpuinfo";
  std::ifstream file(path);

  if (is_readable(path)) {
    std::map<std::string, std::string> map;

    for (std::string line; std::getline(file, line);) {
      /* Empty lines in cpuinfo separate values for individual cpus. */
      if (line.empty()) {
        output.push_back(map);  // Push the current map to the output vector.
        map.clear();            // Clear the contents of the current map.
      } else {
        map.insert({get_key(line), get_value(line)});
      }
    }
  }

  return output;
}

std::optional<std::map<std::string, std::string>> get_cpuinfo_as_map(
    unsigned cpu) {
  if (cpu >= get_cpu_count()) { return {}; }

  /* Static to make subsequent accesses faster, the function is only meant for
   * the invariant parts of cpuinfo. */
  static auto map  = get_complete_cpuinfo_as_map_array();
  static auto size = map.size();

  if (size == 1) {
    return map.at(0);
  } else if (size != get_cpu_count()) {
    /* @todo Handling cases, where the entries in cpuinfo are more/less than the
     * cores available, requires additional parsing. */
    return {};
  } else {
    return map.at(cpu);
  }
}

std::optional<std::string> get_cpuinfo_value(unsigned cpu,
                                             const std::string& key) {
  auto map = get_cpuinfo_as_map(cpu);
  if (!map) return {};
  if (map.value().empty()) return {};

  std::string value;

  try {
    value = map.value().at(key);
  } catch (const std::out_of_range& e) {
    /* @todo Somehow the arm cross-compiler does not respect this try-block, and
     * throws an instance of std::out_of_range when accessing a key that does
     * not exist. */
    return {};
  }

  return value;
}

unsigned get_cpu_count() {
  static auto count =
      static_cast<unsigned>(std::thread::hardware_concurrency());
  return count;
}

unsigned get_thread_count() {
  static auto count =
      static_cast<unsigned>(std::thread::hardware_concurrency());
  return count;
}

unsigned get_core_count() {
  std::vector<unsigned> thread_siblings;
  for (unsigned idx{0}; idx < get_cpu_count(); ++idx) {
    if (auto siblings_count = get_thread_siblings_count(idx))
      thread_siblings.push_back(siblings_count.value());
  }

  /* The core count will only be meaningful if all cpus have the same number of
   * thread siblings. To the best of my knowledge hyperthreading only considers
   * 2 threads per core. */
  if (std::equal(thread_siblings.begin(), thread_siblings.end(),
                 thread_siblings.begin())) {
    /* The assumption is that the values in thread_siblings are powers of 2. */
    return thread_siblings.size() / thread_siblings.at(0);
  } else {
    return 0;
  }
}

unsigned get_socket_count() {
  std::vector<unsigned> package_ids;
  for (unsigned idx{0}; idx < get_cpu_count(); ++idx) {
    if (auto id = get_physical_package_id(idx))
      package_ids.push_back(id.value());
  }

  /* Deduplicate the vector with package ids, the number of unique ids should
   * reflect the socket count on the machine. In the case of ARM big.LITTLE
   * chips the two processors will be identified as distinct sockets. */
  std::sort(package_ids.begin(), package_ids.end());
  package_ids.erase(std::unique(package_ids.begin(), package_ids.end()),
                    package_ids.end());

  return package_ids.size();
}

unsigned get_thread_per_core() {
  /* The assumption is that these values will produce integer results. */
  return get_thread_count() / get_core_count();
}

unsigned get_core_per_socket() {
  /* The assumption is that these values will produce integer results. */
  return get_core_count() / get_socket_count();
}

const char* get_target_architecture() noexcept {
#if defined(__i386__) || defined(__i386)
  return "x86";
#elif defined(__x86_64__)
  return "x86_64";
#elif defined(__ia64__) || defined(__itanium__)
  return "IA-64";
#elif defined(__sparc)
  return "SPARC";
#elif (defined(__arm64__) || defined(__aarch64__))
  return "aarch64";
#elif defined(__arm__)
  return "arm";
#else
  return "unknown";
#endif
}

std::optional<std::string> get_cpu_vendor(unsigned cpu) {
  if (cpu >= get_cpu_count()) { return {}; }

#if defined(__x86_64__)
  return get_cpuinfo_value(cpu, "vendor_id");
#elif defined(__arm__) || defined(__aarch64__)
  /* On ARM chips the vendor will be decoded from a value stored in
   * "CPU implementer". */
  auto implementer = get_cpuinfo_value(cpu, "CPU implementer");
  if (implementer) {
    switch (read_hex_value(implementer.value())) {
      // clang-format off
      case 0x41: return "ARM";
      case 0x42: return "Broadcom";
      case 0x43: return "Cavium";
      case 0x44: return "DEC";
      case 0x4e: return "Nvidia";
      case 0x50: return "APM";
      case 0x51: return "Qualcomm";
      case 0x53: return "Samsung";
      case 0x56: return "Marvell";
      case 0x66: return "Faraday";
      case 0x69: return "Intel";
      default:   return {};  // clang-format on
    }
  }
#endif
  return {};
}

std::optional<std::string> get_cpu_model(unsigned cpu) {
  if (cpu >= get_cpu_count()) { return {}; }

  /* The way in which the cpu model is reported will differ depending on the
   * architecture. */
#if defined(__x86_64__)
  return fmt::format(
      "name: \"{}\", vendor: \"{}\", family: {}, model: {}, stepping: {}",
      get_cpuinfo_value(cpu, "model name").value_or(""),
      get_cpu_vendor(cpu).value_or(""),
      get_cpuinfo_value(cpu, "cpu family").value_or(""),
      get_cpuinfo_value(cpu, "model").value_or(""),
      get_cpuinfo_value(cpu, "stepping").value_or(""));
#elif defined(__arm__) || defined(__aarch64__)
  auto get_part = [](const std::string& str) -> std::optional<std::string> {
    if (str.empty()) {
      return {};
    } else {
      /* For the most common ARM implementer the hexadecimal value will be
       * transformed into a more meaningful name. */
      switch (read_hex_value(str)) {
        // clang-format off
        case 0xc05: return "Cortex-A5";
        case 0xc07: return "Cortex-A7";
        case 0xc08: return "Cortex-A8";
        case 0xc09: return "Cortex-A9";
        case 0xc0d: return "Cortex-A17";
        case 0xc0f: return "Cortex-A15";
        case 0xc0e: return "Cortex-A17";
        case 0xc14: return "Cortex-R4";
        case 0xc15: return "Cortex-R5";
        case 0xc17: return "Cortex-R7";
        case 0xc18: return "Cortex-R8";
        case 0xc20: return "Cortex-M0";
        case 0xc21: return "Cortex-M1";
        case 0xc23: return "Cortex-M3";
        case 0xc24: return "Cortex-M4";
        case 0xc27: return "Cortex-M7";
        case 0xc60: return "Cortex-M0+";
        case 0xd01: return "Cortex-A32";
        case 0xd03: return "Cortex-A53";
        case 0xd04: return "Cortex-A35";
        case 0xd05: return "Cortex-A55";
        case 0xd07: return "Cortex-A57";
        case 0xd08: return "Cortex-A72";
        case 0xd09: return "Cortex-A73";
        case 0xd0a: return "Cortex-A75";
        case 0xd13: return "Cortex-R52";
        case 0xd20: return "Cortex-M23";
        case 0xd21: return "Cortex-M33";
        default:    return str;  // clang-format on
      }
    }
  };

  return fmt::format(
      "arch: {}, vendor: \"{}\", variant: {}, part: \"{}\", revision: {}",
      get_cpuinfo_value(cpu, "CPU architecture").value_or(""),
      get_cpu_vendor(cpu).value_or(""),
      get_cpuinfo_value(cpu, "CPU variant").value_or(""),
      get_part(get_cpuinfo_value(cpu, "CPU part").value_or("")).value_or(""),
      get_cpuinfo_value(cpu, "CPU revision").value_or(""));
#endif
  return {};
}

std::optional<std::vector<std::string>> get_unique_cpu_models() {
  std::vector<std::string> output;
  std::vector<unsigned> cores;
  auto previous = get_cpu_model(0);

  /* Loop through cpus, keep track of which cores share the same model, push
   * when a different model appears. */
  for (unsigned idx{0}; idx < get_cpu_count(); ++idx) {
    auto current = get_cpu_model(idx);
    if (current && previous) {
      if (current.value() != previous.value()) {
        output.push_back(
            fmt::format("cpus: [{}], {}", cores, previous.value()));
        cores.clear();
      }

      cores.push_back(idx);
      previous = current;
    } else {
      return {};
    }
  }

  output.push_back(fmt::format("cpus: [{}], {}", cores, previous.value()));
  return output;
}

std::optional<std::string> get_cpu_flags(unsigned cpu) {
  if (cpu >= get_cpu_count()) { return {}; }

  std::optional<std::string> flags;

  /* @todo Although the function get_cpuinfo_value will return a nullopt for a
   * non-existent key, for some reason compiling for ARM will produce an
   * out_of_range exception, even though there is a try-block in the relevant
   * part of the code. */
#if defined(__x86_64__)
  flags = get_cpuinfo_value(cpu, "flags");
#elif defined(__arm__) || defined(__aarch64__)
  flags = get_cpuinfo_value(cpu, "Features");
#endif
  return flags;
}

std::optional<std::vector<std::string>> get_cpu_flags_array(unsigned cpu) {
  if (auto flags = get_cpu_flags(cpu); flags.has_value()) {
    return split_string(flags.value(), ' ');
  } else {
    return {};
  }
}

std::optional<std::string> get_scaling_governor(unsigned cpu) {
  if (cpu >= get_cpu_count()) { return {}; }

  return get_string_value_from_file(fmt::format(
      "/sys/devices/system/cpu/cpu{}/cpufreq/scaling_governor", cpu));
}

std::optional<unsigned long> get_min_cpu_frequency(unsigned cpu) {
  if (cpu >= get_cpu_count()) { return {}; }

  if (auto frequency = get_string_value_from_file(fmt::format(
          "/sys/devices/system/cpu/cpu{}/cpufreq/cpuinfo_min_freq", cpu))) {
    return std::stoul(frequency.value()) * 1000ul;
  } else {
    return {};
  }
}

std::optional<unsigned long> get_max_cpu_frequency(unsigned cpu) {
  if (!is_cpu_online(cpu) || cpu >= get_cpu_count()) { return {}; }

  if (auto frequency = get_string_value_from_file(fmt::format(
          "/sys/devices/system/cpu/cpu{}/cpufreq/cpuinfo_max_freq", cpu))) {
    return std::stoul(frequency.value()) * 1000ul;
  } else {
    return {};
  }
}

std::optional<std::array<unsigned long, 2>> get_cpu_frequencies(unsigned cpu) {
  if (!is_cpu_online(cpu) || cpu >= get_cpu_count()) { return std::nullopt; }

  auto min = get_min_cpu_frequency(cpu);
  auto max = get_max_cpu_frequency(cpu);

  if (!(min && max)) return std::nullopt;

  return {{min.value(), max.value()}};
}

std::string get_online_cpus() {
  return get_string_value_from_file("/sys/devices/system/cpu/online")
      .value_or("");
}

std::string get_offline_cpus() {
  return get_string_value_from_file("/sys/devices/system/cpu/offline")
      .value_or("");
}

std::string get_possible_cpus() {
  return get_string_value_from_file("/sys/devices/system/cpu/possible")
      .value_or("");
}

std::vector<bool> get_online_cpus_array() {
  std::vector<bool> output;

  for (unsigned idx{0}; idx < get_cpu_count(); ++idx) {
    try {
      output.push_back(is_cpu_online(idx));
    } catch (const std::exception& e) {
      /* In case an unsuccessful access to cpu/online fds, clear the output and
       * return an empty vector. This should be enough of an indication that the
       * operation was not completed. */
      output.clear();
      break;
    }
  }

  return output;
}

bool is_cpu_online(unsigned cpu) {
  if (cpu >= get_cpu_count()) { return false; }

  auto online_fd = fmt::format("/sys/devices/system/cpu/cpu{}/online", cpu);
  auto readable  = is_readable(online_fd);

  /* On some machines, especially Intel, cpu0 does not have an /online fd in
   * sysfs. Therefore we need to parse the complete online description to
   * determine if cpu0 is online. */
  if (!readable && cpu == 0) {
    auto tmp = get_string_value_from_file("/sys/devices/system/cpu/online");
    if (!tmp) {
      /* If the fallback fd is still missing, the error is unrecoverable. */
      throw std::logic_error("sysfs cpu/online is not accessible");
    } else if (tmp.value().empty()) {
      /* There should be some value reported by the cpu/online fd. */
      throw std::logic_error("sysfs cpu/online string is empty");
    } else {
      if (tmp.value()[0] == '0') {
        return true;
      } else {
        return false;
      }
    }
  } else if (!readable) {
    /* Since out of range errors are taken care of, an inaccessible fd is an
     * unrecoverable error. */
    throw std::logic_error("sysfs cpu/online is not readable");
  } else {
    std::ifstream stream{online_fd};
    bool tmp;
    stream >> tmp;
    return tmp;
  }
}

std::optional<std::string> get_topology_value(unsigned cpu,
                                              const std::string& key) {
  if (cpu >= get_cpu_count() || !is_cpu_online(cpu)) { return {}; }

  /* Only accept valid keys, some of which might not exist on all machines, in
   * particular the book_id, drawer_id, and book_siblings, drawer_siblings. */
  if (key == "core_id" || key == "core_siblings" ||
      key == "physical_package_id" || key == "thread_siblings" ||
      key == "core_siblings_list" || key == "thread_siblings_list" ||
      key == "book_id" || key == "drawer_id" || key == "book_siblings" ||
      key == "drawer_siblings") {
    /* get_string_value_from_file can still return nullopt. */
    return get_string_value_from_file(
        fmt::format("/sys/devices/system/cpu/cpu{}/topology/{}", cpu, key));
  } else {
    return {};
  }
}

std::optional<unsigned> get_core_id(unsigned cpu) {
  if (cpu >= get_cpu_count() || !is_cpu_online(cpu)) { return {}; }

  if (auto core_id = get_topology_value(cpu, "core_id")) {
    return static_cast<unsigned>(std::stoul(core_id.value()));
  } else {
    return {};
  }
}

std::optional<unsigned> get_core_siblings(unsigned cpu) {
  if (cpu >= get_cpu_count() || !is_cpu_online(cpu)) { return {}; }

  if (auto core_siblings = get_topology_value(cpu, "core_siblings")) {
    return read_hex_value(split_string(core_siblings.value(), ',').back());
  } else {
    return {};
  }
}

std::optional<std::vector<unsigned>> get_core_siblings_array(unsigned cpu) {
  std::vector<unsigned> siblings;

  auto core_siblings = get_core_siblings(cpu);
  if (!core_siblings) { return std::nullopt; }

  auto value = core_siblings.value();

  unsigned idx{0};
  while (value != 0) {
    if (value & 1) { siblings.push_back(static_cast<unsigned>(idx)); }

    ++idx;
    value >>= 1;
  }

  if (siblings.empty()) {
    return std::nullopt;
  } else {
    return siblings;
  }
}

std::optional<unsigned> get_core_siblings_count(unsigned cpu) {
  if (auto core_siblings = get_core_siblings(cpu)) {
    return hamming_weight(core_siblings.value());
  } else {
    return {};
  }
}

std::optional<unsigned> get_physical_package_id(unsigned cpu) {
  if (cpu >= get_cpu_count() || !is_cpu_online(cpu)) { return {}; }

  if (auto physical_package_id =
          get_topology_value(cpu, "physical_package_id")) {
    return static_cast<unsigned>(std::stoul(physical_package_id.value()));
  } else {
    return {};
  }
}

std::optional<unsigned> get_thread_siblings(unsigned cpu) {
  if (cpu >= get_cpu_count() || !is_cpu_online(cpu)) { return {}; }

  if (auto thread_siblings = get_topology_value(cpu, "thread_siblings")) {
    return read_hex_value(split_string(thread_siblings.value(), ',').back());
  } else {
    return {};
  }
}

std::optional<std::vector<unsigned>> get_thread_siblings_array(unsigned cpu) {
  std::vector<unsigned> siblings;

  auto thread_siblings = get_thread_siblings(cpu);
  if (!thread_siblings) { return std::nullopt; }

  auto value = thread_siblings.value();

  unsigned idx{0};
  while (value != 0) {
    if (value & 1) { siblings.push_back(static_cast<unsigned>(idx)); }

    ++idx;
    value >>= 1;
  }

  if (siblings.empty()) {
    return std::nullopt;
  } else {
    return siblings;
  }
}

std::optional<unsigned> get_thread_siblings_count(unsigned cpu) {
  /* The values in *_siblings are bitsets, therefore the hamming weight gives
   * the total count of 1's in the bitset. */
  if (auto thread_siblings = get_thread_siblings(cpu)) {
    return hamming_weight(thread_siblings.value());
  } else {
    return {};
  }
}

std::optional<std::map<std::string, std::string>> get_cpu_topology_map(
    unsigned cpu) {
  if (cpu >= get_cpu_count() || !is_cpu_online(cpu)) { return std::nullopt; }

  std::map<std::string, std::string> map;

  auto topo_fds = list_directory(
      fmt::format("/sys/devices/system/cpu/cpu{}/topology", cpu));
  /* Remove the "." and ".." directories from the listing. */
  topo_fds.erase(topo_fds.begin(), topo_fds.begin() + 2);

  for (decltype(topo_fds)::iterator it = topo_fds.begin(); it != topo_fds.end();
       ++it) {
    std::ifstream stream{*it};
    std::string value;
    stream >> value;

    map.insert({split_string(*it, '/').back(), value});
  }

  return map;
}  // namespace exot::utilities

std::optional<std::string> get_cpu_topology(unsigned cpu) {
  std::string description;

  if (cpu >= get_cpu_count() || !is_cpu_online(cpu)) { return {}; }

  std::vector<std::string> topo_fds;

  try {
    topo_fds = list_directory(
        fmt::format("/sys/devices/system/cpu/cpu{}/topology", cpu));
  } catch (const std::exception& e) {
    /* Directory not accessible. */
    return {};
  }

  if (topo_fds.empty()) return {};

  /* Remove the "." and ".." directories from the listing. */
  topo_fds.erase(topo_fds.begin(), topo_fds.begin() + 2);

  description = "{ ";

  for (decltype(topo_fds)::iterator it = topo_fds.begin(); it != topo_fds.end();
       ++it) {
    std::ifstream stream{*it};
    std::string value;
    stream >> value;

    description +=
        it != topo_fds.end() - 1
            ? fmt::format("{}: {}, ", split_string(*it, '/').back(), value)
            : fmt::format("{}: {} }}", split_string(*it, '/').back(), value);
  }

  return description;
}

std::vector<std::string> get_complete_topology() {
  std::string sysfs_dir{"/sys/devices/system/cpu"};
  std::string regex{".*/cpu/cpu\\d+"};
  std::vector<std::string> descriptions;

  auto dirs = grep_directory_r(sysfs_dir, regex);

  unsigned idx{0};
  for (const auto& dir : dirs) {
    try {
      descriptions.push_back(fmt::format(
          "{{ core: {}, online: {}, topology: {} }}", idx, is_cpu_online(idx),
          get_cpu_topology(idx).value_or("{}")));
      ++idx;
    } catch (std::logic_error& e) {
      descriptions.clear();
      break;
    }
  }

  return descriptions;
}

}  // namespace exot::utilities
