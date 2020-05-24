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
 * @file primitives/x86_64.cpp
 * @author     Bruno Klopott
 * @brief      Implementation of the IA-64 specific low level utilities.
 */

#if defined(__x86_64__)

#include <exot/primitives/x86_64.h>
#include <fmt/format.h>

namespace exot::primitives {

inline namespace x86_64 {
std::array<std::uint32_t, 4> cpuid(std::uint32_t eax, std::uint32_t ecx) {
  std::uint32_t ebx, edx;

/* Check used for position-independent code compilation in a 2013 document from
 * Intel Corporation "How to detect New Instruction support in the 4th
 * generation Intel® CoreTM processor family" by Max Locktyukhin.
 *
 * For example, PIC has to be used when compiling with Android NDK. */
#if defined(__PIC__)
  asm volatile(
      "movl %%ebx, %%edi \n"
      "cpuid \n"
      "xchgl %%ebx, %%edi"
      : "=D"(ebx),
#else
  asm volatile("cpuid"
               : "+b"(ebx),
#endif
        "+a"(eax), "+c"(ecx), "=d"(edx));

#ifndef DEBUG
  /* @todo This statement is used temporarily to solve an unexpected segfault
   * "Access not within mapped region" in modules/thermal_msr.cpp, when compiled
   * with GCC 7.3.0 with optimisation levels 2 and 3 in build configurations
   * Release or RelWithDebInfo */
  static_cast<void>(fmt::format("{},{},{},{}", eax, ebx, ecx, edx));
#endif
  return {eax, ebx, ecx, edx};
}

unsigned int logical_id() {
  /* If the variable had only the `static` storage specifier, the value would be
   * 'cached' over all invocations of the function, regardless of which thread
   * on which logical processor executed the function. However, if the variable
   * is marked further with the `thread_local` specifier, the variable is
   * allocated when a thread begins and deallocated when it ends, therefore each
   * thread has its own 'cached' instance of the cpuid information.
   *
   * It is better to cache the results of the CPUID instruction, because it can
   * take multiple clock cycles and incurs a large latency in the dependency
   * chain.
   *
   * For example, a quick informal benchmark for 100'000 `logical_id()`
   * invocations results in execution times of 850µs for the code below, and
   * 11500µs for a non-caching version, three orders of magnitude more. */
  static thread_local auto cpuid_0x0B{cpuid(0x0B, 0)};
  return static_cast<unsigned int>(cpuid_0x0B[3] & 0xFFFFUL);
}

unsigned int cluster_id() {
  static thread_local auto cpuid_0x0B{cpuid(0x0B, 0)};
  return static_cast<unsigned int>((cpuid_0x0B[3] >> 16) & 0xFFFFUL);
}

unsigned int logical_processors_per_core() {
  static thread_local auto cpuid_0x0B{cpuid(0x0B, 0)};
  return static_cast<unsigned int>(cpuid_0x0B[1]);
}

}  // namespace x86_64

}  // namespace exot::primitives

#endif
