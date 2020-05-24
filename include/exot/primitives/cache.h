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
 * @file primitives/cache.h
 * @author     Bruno Klopott
 * @brief      Primitives and utilities for cache-based channels.
 */

#pragma once

#ifndef ALWAYS_INLINE
#define ALWAYS_INLINE inline __attribute__((always_inline))
#endif

#include <cstdint>
#include <limits>
#include <type_traits>

#include <exot/utilities/bits.h>

namespace exot::primitives {

/* x86 primitives. */

#if defined(__x86_64__)

inline namespace x86_64 {
/**
 * @brief      Reads an address into a temporary register
 * @note       Will use instructions specific to 32-bit and 64-bit integers if
 *             such are provided. Clobbers the temporary register, "RAX" or
 *             "EAX".
 *
 * @param      address    The address
 *
 * @tparam     S          An integer for size information
 * @tparam     <unnamed>  Template helper
 */
template <
    typename S = std::uint32_t,
    typename   = std::enable_if_t<exot::utilities::are_unsigned_integral<S>>>
ALWAYS_INLINE void access_read(void* address) {
  static_assert(std::numeric_limits<S>::digits <= 64,
                "Only fundamental types are allowed.");

  if constexpr (std::numeric_limits<S>::digits == 64) {
    asm volatile(                                   //
        "movq (%0), %%rax      # EXOT->cache     "  //
        :                                           // output
        : "r"(address)                              // input
        : "%rax");                                  // clobber
  } else {
    asm volatile(                                   //
        "mov  (%0), %%eax      # EXOT->cache     "  //
        :                                           // output
        : "r"(address)                              // input
        : "%eax");                                  // clobber
  }
}

/**
 * @brief      Writes an immediate value to an address
 * @note       Will use instructions specific to 32-bit and 64-bit integers if
 *             such are provided.
 *
 * @param      address    The address
 *
 * @tparam     S          An integer for size information
 * @tparam     <unnamed>  Template helper
 */
template <
    typename S = std::uint32_t,
    typename   = std::enable_if_t<exot::utilities::are_unsigned_integral<S>>>
ALWAYS_INLINE void access_write(void* address) {
  static_assert(std::numeric_limits<S>::digits <= 64,
                "Only fundamental types are allowed.");

  if constexpr (std::numeric_limits<S>::digits == 64) {
    asm volatile(                                    //
        "movq $0, %0           # EXOT->cache      "  //
        :                                            // output
        : "r"(address)                               // input
        :);                                          // clobber
  } else {
    asm volatile(                                    //
        "mov  $0, %0           # EXOT->cache      "  //
        :                                            // output
        : "r"(address)                               // input
        :);                                          // clobber
  }
}

/**
 * @brief      Instructs the processor to prefetch an address
 *
 * @param      address  The address
 */
ALWAYS_INLINE void prefetch(void* address) {
  asm volatile(
      "prefetchnta (%0)        # EXOT->cache \n\t"  //
      "prefetcht2 (%0)         # EXOT->cache     "  //
      :                                             // output
      : "r"(address));                              // input
}

/**
 * @brief      Instructs the processor to flush an address from all caches
 * @details    "Executions of the CLFLUSH instruction are ordered with respect
 *             to each other and with respect to writes, locked
 *             read-modify-write instructions, fence instructions, and
 *             executions of CLFLUSHOPT to the same cache line." Also,
 *             "CLFLUSH instruction may always cause transactional abort with
 *             Transactional Synchronization Extensions (TSX)."
 *             See: https://www.felixcloutier.com/x86/clflush.
 *
 * @param      address  The address
 */
ALWAYS_INLINE void flush(void* address) {
  asm volatile(                                     //
      "clflush 0(%0)           # EXOT->cache     "  //
      :                                             // output
      : "r"(address));                              // input
}

ALWAYS_INLINE void flushopt(void* address) {
  asm volatile(                                     //
      "clflushopt 0(%0)        # EXOT->cache     "  //
      :                                             // output
      : "r"(address));                              // input
}

/**
 * @brief      Measures access time to an address and flushes it
 * @note       Function as given in the FLUSH+RELOAD paper [1].
 *
 *             [1]  Y. Yarom and K. Falkner, “FLUSH+RELOAD - A High Resolution,
 *             Low Noise, L3 Cache Side-Channel Attack.,” 23rd USENIX Security
 *             Symposium, 2014.
 *
 * @param      address  The address
 *
 * @return     The time to access the address in cycles
 */
ALWAYS_INLINE auto flush_reload(void* address) {
  volatile std::uint32_t value;

  asm volatile(
      "mfence                  # EXOT->cache \n\t"  //
      "lfence                  # EXOT->cache \n\t"  //
      "rdtsc                   # EXOT->cache \n\t"  // saves in edx:eax
      "lfence                  # EXOT->cache \n\t"  //
      "movl %%eax, %%esi       # EXOT->cache \n\t"  // save eax of 1st rdtsc
      "movl (%1), %%eax        # EXOT->cache \n\t"  // access address (ecx)
      "lfence                  # EXOT->cache \n\t"  //
      "rdtsc                   # EXOT->cache \n\t"  //
      "subl %%esi, %%eax       # EXOT->cache \n\t"  // 2nd rdtsc - 1st rdtsc
      "clflush 0(%1)           # EXOT->cache     "  //
      : "=a"(value)                                 // output
      : "c"(address)                                // input
      : "%esi", "%edx");                            // clobber

  return value;
}

/**
 * The following values are codify the bit positions that are xor'ed to
 * produce a hash bit in the reverse-engineered Intel slice hashing function,
 * as given in [1].
 *
 * [1]  C. Maurice, N. Le Scouarnec, C. Neumann, O. Heen, and A. Francillon,
 * “Reverse Engineering Intel Last-Level Cache Complex Addressing Using
 * Performance Counters.,” RAID, vol. 9404, no. 3, pp. 48–65, 2015.
 *
 *                                                          tag + index | off
 * Bit position:                        3333333322222222221111111111
 *                                      76543210987654321098765432109876543210
 */
static constexpr std::uintptr_t O_0 = 0b01101101011111010101110101010001000000;
static constexpr std::uintptr_t O_1 = 0b10111010110101111110101010100010000000;
static constexpr std::uintptr_t O_2 = 0b11110011001100110010010011000100000000;

/**
 * @brief      Slice selection hash function for Intel Sandy Bridge, Ivy Bridge,
 *             and Haswell.
 * @note       Based on the reverse engineering done by Maurice et al. (2015).
 *
 * @param[in]  physical_address  The physical address
 * @param[in]  slice_count       The number of slices in the system
 *
 * @return     The LLC slice in which the physical address is located
 */
constexpr inline auto slice_selection_hash(std::uintptr_t physical_address,
                                           unsigned slice_count)
    -> unsigned int {
  using namespace exot::utilities;  // for parity

  if (slice_count == 2) {                            // ------
    return (parity(physical_address & O_0))          // bit 0
           & 0b1;                                    // mask
  } else if (slice_count == 4) {                     // ------
    return ((parity(physical_address & O_0)) |       // bit 0
            (parity(physical_address & O_1) << 1))   // bit 1
           & 0b11;                                   // mask
  } else if (slice_count == 8) {                     // ------
    return ((parity(physical_address & O_0)) |       // bit 0
            (parity(physical_address & O_1) << 1) |  // bit 1
            (parity(physical_address & O_2) << 2))   // bit 2
           & 0b111;                                  // mask
  } else {
    return -1;
  }
}

}  // namespace x86_64

#endif

/* ARMv8 primitives. */

#if defined(__aarch64__)

inline namespace aarch64 {
/**
 * @brief      Reads an address into a temporary register
 *
 * @param      address    The address
 *
 * @tparam     S          An integer for size information
 * @tparam     <unnamed>  Template helper
 */
template <
    typename S = std::uint32_t,
    typename   = std::enable_if_t<exot::utilities::are_unsigned_integral<S>>>
ALWAYS_INLINE void access_read(void* address) {
  static_assert(std::numeric_limits<S>::digits <= 64,
                "Only fundamental types are allowed.");
  volatile S value;

  if constexpr (std::numeric_limits<S>::digits == 8) {
    asm volatile(                                    //
        "ldrb %0, [%1]         // EXOT->cache     "  //
        : "=r"(value)                                // output
        : "r"(address)                               // input
        :);                                          // clobber
  } else if constexpr (std::numeric_limits<S>::digits == 16) {
    asm volatile(                                    //
        "ldrh %0, [%1]         // EXOT->cache     "  //
        : "=r"(value)                                // output
        : "r"(address)                               // input
        :);                                          // clobber
  } else {
    asm volatile(                                    //
        "ldr %0, [%1]          // EXOT->cache     "  //
        : "=r"(value)                                // output
        : "r"(address)                               // input
        :);                                          // clobber
  }
}

/**
 * @brief      Writes an immediate value to an address
 * @note       Will use instructions specific to 32-bit and 64-bit integers if
 *             such are provided.
 *
 * @param      address    The address
 *
 * @tparam     S          An integer for size information
 * @tparam     <unnamed>  Template helper
 */
template <
    typename S = std::uint32_t,
    typename   = std::enable_if_t<exot::utilities::are_unsigned_integral<S>>>
ALWAYS_INLINE void access_write(void* address) {
  static_assert(std::numeric_limits<S>::digits <= 64,
                "Only fundamental types are allowed.");
  volatile S value;

  if constexpr (std::numeric_limits<S>::digits == 8) {
    asm volatile(                                    //
        "ldrb %0, [%1]         // EXOT->cache     "  //
        : "=r"(value)                                // output
        : "r"(address)                               // input
        :);                                          // clobber
  } else if constexpr (std::numeric_limits<S>::digits == 16) {
    asm volatile(                                    //
        "ldrh %0, [%1]         // EXOT->cache     "  //
        : "=r"(value)                                // output
        : "r"(address)                               // input
        :);                                          // clobber
  } else {
    asm volatile(                                    //
        "ldr %0, [%1]          // EXOT->cache     "  //
        : "=r"(value)                                // output
        : "r"(address)                               // input
        :);                                          // clobber
  }
}

/**
 * @brief      Instructs the processor to prefetch an address
 * @note       Uses the following operations:
 *
 *             Instruction breakdown [prfm|pld|l1|keep]:
 *             - [prfm]: Prefetch from memory
 *               - [pld]: Prefetch for load (as opposed to store with [pst]).
 *                 - [l{1,2,3}]: The cache to target (L1, L2, or L3)
 *                   - [keep]: Policy -> keep in cache.
 *
 *             Enable level 3 by prepending the list with:
 *             "prfm pldl3keep, [%x0]  // EXOT->cache \n\t"
 *
 * @param      address  The address
 */
ALWAYS_INLINE void prefetch(void* address) {
  asm volatile(                                     //
      "prfm pldl2keep, [%x0]  // EXOT->cache \n\t"  //
      "prfm pldl1keep, [%x0]  // EXOT->cache     "  //
      :                                             // output
      : "r"(address));                              // input
}

/**
 * @brief      Instructs the processor to flush an address from all caches
 * @note       Uses the following operations:
 *
 *             - [dc civac]: Data cache clean & invalidate.
 *               - [dc] data cache,
 *               - [civac] option (clean and invalidate by virtual address to
 *                 point of coherency).
 *             - [dsb ish]: Ensure completion of invalidations.
 *               - [dsb] data synchronisation barrier,
 *               - [ish] option (inner shareable: ordered access before and
 *                 after any load/store combination).
 *             - [isb]: Synchronise the fetched instruction stream.
 *               - [isb] instruction synchronization barrier.
 *
 * @param      address  The address
 */
ALWAYS_INLINE void flush(void* address) {
  asm volatile(                                      //
      "dc civac, %0            // EXOT->cache \n\t"  // clean & invalidate
      "dsb ish                 // EXOT->cache \n\t"  // ensure completion
      "isb                     // EXOT->cache     "  // synchronise instrs.
      :                                              // output
      : "r"(address));                               // input
}

}  // namespace aarch64

#endif

#if defined(__arm__) && !defined(__aarch64__)

inline namespace arm {
/**
 * @brief      Reads an address into a temporary register
 *
 * @param      address    The address
 *
 * @tparam     S          An integer for size information
 * @tparam     <unnamed>  Template helper
 */
template <
    typename S = std::uint32_t,
    typename   = std::enable_if_t<exot::utilities::are_unsigned_integral<S>>>
ALWAYS_INLINE void access_read(void* address) {
  static_assert(std::numeric_limits<S>::digits <= 64,
                "Only fundamental types are allowed.");
  volatile S value;

  if constexpr (std::numeric_limits<S>::digits == 8) {
    asm volatile(                                    //
        "ldrb %0, [%1]         // EXOT->cache     "  //
        : "=r"(value)                                // output
        : "r"(address)                               // input
        :);                                          // clobber
  } else if constexpr (std::numeric_limits<S>::digits == 16) {
    asm volatile(                                    //
        "ldrh %0, [%1]         // EXOT->cache     "  //
        : "=r"(value)                                // output
        : "r"(address)                               // input
        :);                                          // clobber
  } else {
    asm volatile(                                    //
        "ldr %0, [%1]          // EXOT->cache     "  //
        : "=r"(value)                                // output
        : "r"(address)                               // input
        :);                                          // clobber
  }
}

/**
 * @brief      Writes an immediate value to an address
 * @note       Will use instructions specific to 32-bit and 64-bit integers if
 *             such are provided.
 *
 * @param      address    The address
 *
 * @tparam     S          An integer for size information
 * @tparam     <unnamed>  Template helper
 */
template <
    typename S = std::uint32_t,
    typename   = std::enable_if_t<exot::utilities::are_unsigned_integral<S>>>
ALWAYS_INLINE void access_write(void* address) {
  static_assert(std::numeric_limits<S>::digits <= 64,
                "Only fundamental types are allowed.");
  volatile S value;

  if constexpr (std::numeric_limits<S>::digits == 8) {
    asm volatile(                                    //
        "ldrb %0, [%1]         // EXOT->cache     "  //
        : "=r"(value)                                // output
        : "r"(address)                               // input
        :);                                          // clobber
  } else if constexpr (std::numeric_limits<S>::digits == 16) {
    asm volatile(                                    //
        "ldrh %0, [%1]         // EXOT->cache     "  //
        : "=r"(value)                                // output
        : "r"(address)                               // input
        :);                                          // clobber
  } else {
    asm volatile(                                    //
        "ldr %0, [%1]          // EXOT->cache     "  //
        : "=r"(value)                                // output
        : "r"(address)                               // input
        :);                                          // clobber
  }
}

/**
 * @brief      Instructs the processor to prefetch an address
 * @note       Uses the following operations:
 *
 *             Instruction breakdown [prfm|pld|l1|keep]:
 *             - [prfm]: Prefetch from memory
 *               - [pld]: Prefetch for load (as opposed to store with [pst]).
 *                 - [l{1,2,3}]: The cache to target (L1, L2, or L3)
 *                   - [keep]: Policy -> keep in cache.
 *
 * @param      address  The address
 */
ALWAYS_INLINE void prefetch(void* address) {
  asm volatile(                                     //
      "prfm pldl3keep, [%x0]  // EXOT->cache \n\t"  //
      "prfm pldl2keep, [%x0]  // EXOT->cache \n\t"  //
      "prfm pldl1keep, [%x0]  // EXOT->cache     "  //
      :                                             // output
      : "r"(address));                              // input
}

}  // namespace arm

#endif

}  // namespace exot::primitives
