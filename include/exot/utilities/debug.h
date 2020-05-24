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
 * @file utilities/debug.h
 * @author     Bruno Klopott
 * @brief      Debugging helpers, in global namespace
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <typeinfo>

#include <fmt/format.h>

#ifdef __GNUG__
#include <cxxabi.h>
#endif  // __GNUG__

#include <exot/utilities/cache.h>
#include <exot/utilities/pagemap.h>

inline namespace debug {
/**
 * @brief      Produces a human-readable type description
 * @details    Demangling in this way works only with the GNU compiler. For
 *             other compilers one can always use the c++filt application.
 *
 * @param[in]  mangled_name  The mangled type name
 *
 * @return     Demangled type name
 */
inline const char* demangle(const char* mangled_name) {
#ifdef __GNUG__
  std::size_t len = 0;
  int status      = 0;
  std::unique_ptr<char, decltype(&std::free)> ptr(
      __cxxabiv1::__cxa_demangle(mangled_name, nullptr, &len, &status),
      &std::free);
  return ptr.get();
#else
  return mangled_name;
#endif
}

template <typename T>
static inline const char* typeid_name = typeid(T).name();

template <typename T>
static inline const char* demangled_typeid_name = demangle(typeid(T).name());

/**
 * @brief      Print memory contents, virtual, and physical addresses
 *
 * @param[in]  header  The header to print
 * @param[in]  base    The base pointer as a value
 * @param[in]  cnt     The count of addresses to print
 * @param[in]  jmp     The jump value by which the addresses are incremented
 */
static inline void print_memory(const char* header, std::uintptr_t base,
                                std::size_t cnt = 1, std::size_t jmp = 1) {
  try {
    auto translator = exot::utilities::AddressTranslator();
    auto decomposer = exot::utilities::AddressDecomposer();

    fmt::print(stderr, "\n-- {}\n\n", header);
    fmt::print(stderr,
               "   idx |             virt >>             phys |~ contents ~|"
               "\ttag\tset\toff\n");
    fmt::print(stderr,
               "-------|--------------------------------------|------------|"
               "-------------------------------\n");
    for (auto i = size_t{0}; i < cnt * jmp; i += jmp) {
      auto* ptr            = reinterpret_cast<void*>(base + i);
      auto* buf            = reinterpret_cast<const unsigned*>(ptr);
      auto phys            = translator(ptr);
      auto [tag, set, off] = decomposer(phys);

      fmt::print(stderr,
                 "{:0>6x} | {:>16} >> {:#16x} |~ {:>{}x} ~|\t"  // 1
                 "{:#x}\t{:#x}\t{:#x}\n",                       // 2
                 i, ptr, phys, buf[0], 2 * sizeof(buf[0]),      // -> 1
                 tag, set, off);                                // -> 2
    }

  } catch (const std::exception& e) {
    fmt::print(stderr, "-- failed: {}", e.what());
  }
}

}  // namespace debug
