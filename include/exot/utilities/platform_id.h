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
 * @file utilities/platform_id.h
 * @author     Bruno Klopott
 * @brief      Platform identification utilities.
 *
 * @todo       Abstract discovery into a JSON-serialisable class
 */

#pragma once

#include <array>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace exot::utilities {

/**
 * @brief      Gets the kernel version.
 *
 * @return     The kernel version. Nullopt if unsuccessful.
 */
std::optional<std::string> get_kernel_version();

/**
 * @brief      Gets all the available cpuinfo keys.
 *
 * @return     The cpuinfo keys.
 */
std::vector<std::string> get_cpuinfo_keys();

/**
 * @brief      Gets the cpuinfo key-value pairs as a map for a specific cpu.
 *
 * @param[in]  cpu   The cpu
 *
 * @return     The cpuinfo as map. Empty if unsuccessful.
 */
std::optional<std::map<std::string, std::string>> get_cpuinfo_as_map(
    unsigned cpu);

/**
 * @brief      Gets the complete cpuinfo as map array.
 *
 * @return     The complete cpuinfo as map array.
 */
std::vector<std::map<std::string, std::string>>
get_complete_cpuinfo_as_map_array();

/**
 * @brief      Gets the cpuinfo value for a given key for a specific cpu.
 *
 * @param[in]  cpu   The cpu
 * @param[in]  key   The key
 *
 * @return     The cpuinfo value. Nullopt if unsuccessful.
 */
std::optional<std::string> get_cpuinfo_value(unsigned cpu,
                                             const std::string& key);

/**
 * @brief      Gets the cpuinfo values for a given key for all cpus.
 *
 * @param[in]  key   The key
 *
 * @return     The cpuinfo values. Empty if unsuccessful.
 */
std::vector<std::string> get_cpuinfo_values(const std::string& key);

/**
 * @brief      Gets the cpu count.
 *
 * @return     The cpu count.
 */
unsigned get_cpu_count();

/**
 * @brief      Gets the thread count.
 *
 * @return     The thread count.
 */
unsigned get_thread_count();

/**
 * @brief      Gets the core count.
 *
 * @return     The core count.
 */
unsigned get_core_count();

/**
 * @brief      Gets the socket count.
 *
 * @return     The socket count.
 */
unsigned get_socket_count();

/**
 * @brief      Gets the thread per core. Derived from get_thread_count and
 *             get_core_count.
 *
 * @return     The thread per core.
 */
unsigned get_thread_per_core();

/**
 * @brief      Gets the core per socket. Derived from get_core_count and
 *             get_socket_count.
 *
 * @return     The core per socket.
 */
unsigned get_core_per_socket();

/**
 * @brief      Gets the target architecture. Returns a compile-time value.
 *
 * @return     The target architecture for which the binary was compiled.
 */
const char* get_target_architecture() noexcept;

/**
 * @brief      Gets the cpu vendor of a specific cpu.
 *
 * @param[in]  cpu   The cpu
 *
 * @return     The cpu vendor. Nullopt if unsuccessful, or if provided cpu does
 *             not exist.
 */
std::optional<std::string> get_cpu_vendor(unsigned cpu);

/**
 * @brief      Gets the cpu model of a specific cpu.
 * @details    For x86_64 chips the model will be reported as:
 *                name: "{}", family: {}, model: {}, stepping: {}
 *             For ARM-based chips the model will be reported as:
 *                arch: {}, variant: {}, part: "{}", revision: {}
 *
 * @param[in]  cpu   The cpu
 *
 * @return     The cpu model. Nullopt if unsuccessful, or if provided cpu does
 *             not exist.
 */
std::optional<std::string> get_cpu_model(unsigned cpu);

/**
 * @brief      Gets all unique cpu models and their associated cpus.
 *
 * @return     A vector with unique cpu models and the associated cpu list.
 *             Nullopt if unsuccessful.
 */
std::optional<std::vector<std::string>> get_unique_cpu_models();

/**
 * @brief      Gets the cpu flags of a specific cpu.
 *
 * @param[in]  cpu   The cpu
 *
 * @return     The cpu flags. Nullopt if unsuccessful, or if provided cpu does
 *             not exist.
 */
std::optional<std::string> get_cpu_flags(unsigned cpu);

/**
 * @brief      Gets the cpu flags as an array for a specific cpu.
 *
 * @param[in]  cpu   The cpu
 *
 * @return     The cpu flags array. Nullopt if unsuccessful, or if provided cpu
 *             does not exist.
 */
std::optional<std::vector<std::string>> get_cpu_flags_array(unsigned cpu);

/**
 * @brief      Gets the scaling governor of a specific cpu.
 *
 * @param[in]  cpu   The cpu
 *
 * @return     The scaling governor. Nullopt if unsuccessful, or if provided cpu
 *             does not exist.
 */
std::optional<std::string> get_scaling_governor(unsigned cpu);

/**
 * @brief      Gets the min and max cpu frequencies of a specific cpu. The
 *             frequencies are given in Hz.
 *
 * @param[in]  cpu   The cpu
 *
 * @return     The cpu frequencies. Nullopt if unsuccessful, or if provided cpu
 *             does not exist.
 */
std::optional<std::array<unsigned long, 2>> get_cpu_frequencies(unsigned cpu);

/**
 * @brief      Gets the minimum cpu frequency of a specific cpu. The frequency
 *             is given in Hz.
 *
 * @param[in]  cpu   The cpu
 *
 * @return     The minimum cpu frequency. Nullopt if unsuccessful, or if
 *             provided cpu does not exist.
 */
std::optional<unsigned long> get_min_cpu_frequency(unsigned cpu);

/**
 * @brief      Gets the maximum cpu frequency of a specific cpu. The frequency
 *             is given in Hz.
 *
 * @param[in]  cpu   The cpu
 *
 * @return     The maximum cpu frequency. Nullopt if unsuccessful, or if
 *             provided cpu does not exist.
 */
std::optional<unsigned long> get_max_cpu_frequency(unsigned cpu);

/**
 * @brief      Gets the string representation of online cpus.
 * @details    The string representation usually informs about a range of active
 *             cpus, e.g. in the form "0-7" for active cpus from 0 to 7.
 *
 * @return     The online cpus. An empty string if unsuccessful.
 */
std::string get_online_cpus();

/**
 * @brief      Gets the string representation of offline cpus.
 *
 * @return     The offline cpus. An empty string if unsuccessful.
 */
std::string get_offline_cpus();

/**
 * @brief      Gets the string representation of possible cpus. Possible cpus
 *             are those that can be either brought online, or made offline.
 *
 * @return     The possible cpus. An empty string if unsuccessful.
 */
std::string get_possible_cpus();

/**
 * @brief      Gets the online cpus array.
 *
 * @return     The online cpus array. Empty if unsuccessful.
 */
std::vector<bool> get_online_cpus_array();

/**
 * @brief      Determines if a cpu online.
 *
 * @param[in]  cpu   The cpu
 *
 * @return     True if cpu online, False otherwise. Also false for non-existent
 *             cpus.
 */
bool is_cpu_online(unsigned int cpu);

/**
 * @brief      Gets the topology value given a valid key for a specific cpu.
 * @details    The possible keys are: "core_id", "core_siblings",
 *             "physical_package_id", "thread_siblings", "core_siblings_list",
 *             "thread_siblings_list", "book_id", "drawer_id", "book_siblings",
 *             "drawer_siblings".
 *
 * @param[in]  cpu   The cpu
 * @param[in]  key   The key
 *
 * @return     The topology value. Nullopt if unsuccessful, or if provided cpu
 *             does not exist.
 */
std::optional<std::string> get_topology_value(unsigned cpu,
                                              const std::string& key);

/**
 * @brief      Gets the core identifier of a specific cpu.
 *
 * @param[in]  cpu   The cpu
 *
 * @return     The core identifier.Nullopt if unsuccessful, or if provided cpu
 *             does not exist.
 */
std::optional<unsigned> get_core_id(unsigned cpu);

/**
 * @brief      Gets the core siblings of a specific cpu.
 *
 * @param[in]  cpu   The cpu
 *
 * @return     The core siblings. Nullopt if unsuccessful, or if provided cpu
 *             does not exist.
 */
std::optional<unsigned> get_core_siblings(unsigned cpu);

/**
 * @brief      Gets the core siblings as an array for a specific cpu.
 *
 * @param[in]  cpu   The cpu
 *
 * @return     The core siblings array. Nullopt if unsuccessful, or if provided
 *             cpu does not exist.
 */
std::optional<std::vector<unsigned>> get_core_siblings_array(unsigned cpu);

/**
 * @brief      Gets the core siblings count of a specific cpu.
 *
 * @param[in]  cpu   The cpu
 *
 * @return     The core siblings count. Nullopt if unsuccessful, or if provided
 *             cpu does not exist.
 */
std::optional<unsigned> get_core_siblings_count(unsigned cpu);

/**
 * @brief      Gets the physical package identifier of a specific cpu.
 *
 * @param[in]  cpu   The cpu
 *
 * @return     The physical package identifier. Nullopt if unsuccessful, or if
 *             provided cpu does not exist.
 */
std::optional<unsigned> get_physical_package_id(unsigned cpu);

/**
 * @brief      Gets the thread siblings of a specific cpu.
 *
 * @param[in]  cpu   The cpu
 *
 * @return     The thread siblings. Nullopt if unsuccessful, or if provided cpu
 *             does not exist.
 */
std::optional<unsigned> get_thread_siblings(unsigned cpu);

/**
 * @brief      Gets the thread siblings as an array for a specific cpu.
 *
 * @param[in]  cpu   The cpu
 *
 * @return     The thread siblings array. Nullopt if unsuccessful, or if
 *             provided cpu does not exist.
 */
std::optional<std::vector<unsigned>> get_thread_siblings_array(unsigned cpu);

/**
 * @brief      Gets the thread siblings count of a specific cpu.
 *
 * @param[in]  cpu   The cpu
 *
 * @return     The thread siblings count. Nullopt if unsuccessful, or if
 *             provided cpu does not exist.
 */
std::optional<unsigned> get_thread_siblings_count(unsigned cpu);

/**
 * @brief      Gets the complete cpu topology as a map for a specific cpu.
 *
 * @param[in]  cpu   The cpu
 *
 * @return     The cpu topology map. Nullopt if unsuccessful, or if provided cpu
 *             does not exist.
 */
std::optional<std::map<std::string, std::string>> get_cpu_topology_map(
    unsigned cpu);

/**
 * @brief      Gets the serialised cpu topology of a specific cpu as a string.
 *             The result is JSON-like formatted.
 *
 * @param[in]  cpu   The cpu
 *
 * @return     The cpu topology. Nullopt if unsuccessful, or if provided cpu
 *             does not exist.
 */
std::optional<std::string> get_cpu_topology(unsigned cpu);

/**
 * @brief      Gets a description of the complete topology for all cpus. The
 *             result is JSON-like formatted.
 *
 * @return     The complete topology.
 */
std::vector<std::string> get_complete_topology();

}  // namespace exot::utilities
