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
 * @file utilities/alignment.h
 * @author     Bruno Klopott
 * @brief      An allocator and a type wrapper for explicitly allocated vectors
 *             and simple arithmetic data types.
 */

#pragma once

#include <type_traits>

namespace exot::utilities {

/**
 * @brief      Determines if a number is a valid alignment.
 *
 * @param[in]  value  The alignment
 *
 * @return     True if a value is a power of 2, False otherwise.
 */
constexpr inline bool is_valid_alignment(std::size_t value) noexcept {
  return (value > 0) && ((value & (value - 1)) == 0);
}

/**
 * @brief      Aligned type wrapper
 *
 * @tparam     T          The value type to wrap
 * @tparam     A          The desired alignment
 * @tparam     <unnamed>  Template helper (only fundamental types allowed)
 */
template <typename T, std::size_t A,
          typename = std::enable_if_t<(
              std::is_arithmetic_v<T> || std::is_pointer_v<T> ||
              std::is_same_v<std::remove_cv_t<T>, void*>)>>
struct aligned_t {
  /* Type aliases */
  using value_type = T;
  using this_type  = aligned_t<T, A>;

  /* The aligned value */
  alignas(A) T _value;

  /* Verify that alignment is achieved */
  static_assert(is_valid_alignment(A));

  /**
   * Default constructor
   */
  aligned_t() = default;

  /**
   * @brief      Copy constructor for value types
   *
   * @param      value  The value
   */
  aligned_t(value_type&& value) : _value(value) {}

  /**
   * @brief      Proxy to the value
   */
  operator T() { return _value; }

  /**
   * @brief      Proxy to the pointer to the value
   */
  T* operator&() __attribute__((assume_aligned(A))) { return &_value; }
};

/**
 * @brief      Makes an aligned version of a value
 *
 * @param[in]  t     The value to make aligned
 *
 * @tparam     A     The alignment
 * @tparam     T     The value type
 *
 * @return     An aligned value
 */
template <std::size_t A, typename T>
constexpr auto make_aligned(T t) -> aligned_t<T, A> {
  return aligned_t<T, A>{t};
}

}  // namespace exot::utilities
