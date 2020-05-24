/**
 * @file primitives/x86_64.h
 * @author     Bruno Klopott
 * @brief      Low-level functions relevant on IA-64 architectures.
 */

#pragma once

#if defined(__x86_64__)

#include <array>    // for fixed-size arrays
#include <cstdint>  // for std::uint*

namespace exot::primitives {

inline namespace x86_64 {

/**
 * @brief      Access the CPUID instruction
 *
 * @param[in]  eax   The initial eax register value
 * @param[in]  ecx   The initial ecx register value (rarely used)
 *
 * @return     An array of register values eax, ebx, ecx, edx, holding the
 *             result of the CPUID instruction
 */
std::array<std::uint32_t, 4> cpuid(std::uint32_t eax, std::uint32_t ecx = 0);
/**
 * @brief      Get the logical affinity of the current thread
 * @details    The value returned in CPUID.0BH:EDX reports the "2APIC ID of the
 *             current logical processor" ID, see Extended Topology Enumeration
 *             Leaf in
 *             "Intel® 64 and IA-32 Architectures Software Developer’s Manual",
 *             Volume 2, September 2016, p. 3-195.
 *
 *             The lower 16 bits hold the id of the logical processor in a
 *             cluster.
 *
 * @return     The logical core on which the current thread is running
 */
unsigned int logical_id();

/**
 * @brief      Get the cluster ID of the current thread
 * @details    The upper 16 bits hold the cluster id.
 *
 * @return     The id of the cluster on which the current thread is running
 */
unsigned int cluster_id();

/**
 * @brief      Get the number of logical processors per physical core
 * @details    The value reported in CPUID.0BH:EBX states the number of logical
 *             processors per core/level.
 *
 * @return     The number of logical processors per core
 */
unsigned int logical_processors_per_core();

}  // namespace x86_64

}  // namespace exot::primitives

#endif
