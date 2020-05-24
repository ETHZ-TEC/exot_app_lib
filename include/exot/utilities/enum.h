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
 * @file utilities/enum.h
 * @author     Bruno Klopott
 * @brief      Helpers to allow operations on typed enums.
 */

#pragma once

#include <type_traits>

namespace exot::utilities {

/**
 * @brief      Type trait to enable bit-wise operations for an Enum
 *
 * @tparam     Enum       The enum
 * @tparam     <unnamed>  Template helper (is enum?)
 */
template <typename Enum, typename = std::enable_if_t<std::is_enum<Enum>::value>>
struct enum_operations_enabled : std::false_type {};

/**
 * @brief      Bit-wise "or" operator overload
 *
 * @param[in]  lhs        The left hand side
 * @param[in]  rhs        The right hand side
 *
 * @tparam     Enum       The enum
 * @tparam     <unnamed>  Template helper
 *
 * @return     The or'ed enum result
 */
template <typename Enum,
          typename = std::enable_if_t<(enum_operations_enabled<Enum>::value &&
                                       std::is_enum<Enum>::value)>>
inline constexpr Enum operator|(Enum lhs, Enum rhs) {
  using T = typename std::underlying_type_t<Enum>;
  return static_cast<Enum>(static_cast<T>(lhs) | static_cast<T>(rhs));
}

/**
 * @brief      Bit-wise "or" assignment operator overload
 *
 * @param[in]  lhs        The left hand side
 * @param[in]  rhs        The right hand side
 *
 * @tparam     Enum       The enum
 * @tparam     <unnamed>  Template helper
 *
 * @return     A reference to the or'ed lhs enum
 */
template <typename Enum,
          typename = std::enable_if_t<(enum_operations_enabled<Enum>::value &&
                                       std::is_enum<Enum>::value)>>
inline constexpr Enum& operator|=(Enum lhs, Enum rhs) {
  using T = typename std::underlying_type_t<Enum>;
  lhs     = static_cast<Enum>(static_cast<T>(lhs) | static_cast<T>(rhs));
  return lhs;
}

/**
 * @brief      Bit-wise "and" operator overload
 *
 * @param[in]  lhs        The left hand side
 * @param[in]  rhs        The right hand side
 *
 * @tparam     Enum       The enum
 * @tparam     <unnamed>  Template helper
 *
 * @return     The and'ed enum
 */
template <typename Enum,
          typename = std::enable_if_t<(enum_operations_enabled<Enum>::value &&
                                       std::is_enum<Enum>::value)>>
inline constexpr Enum operator&(Enum lhs, Enum rhs) {
  using T = typename std::underlying_type_t<Enum>;
  return static_cast<Enum>(static_cast<T>(lhs) & static_cast<T>(rhs));
}

/**
 * @brief      Bit-wise "and" assignment operator overload
 *
 * @param[in]  lhs        The left hand side
 * @param[in]  rhs        The right hand side
 *
 * @tparam     Enum       The enum
 * @tparam     <unnamed>  Template helper
 *
 * @return     A reference to the and'ed lhs enum
 */
template <typename Enum,
          typename = std::enable_if_t<(enum_operations_enabled<Enum>::value &&
                                       std::is_enum<Enum>::value)>>
inline constexpr Enum& operator&=(Enum lhs, Enum rhs) {
  using T = typename std::underlying_type_t<Enum>;
  lhs     = static_cast<Enum>(static_cast<T>(lhs) & static_cast<T>(rhs));
  return lhs;
}

/**
 * @brief      Bit-wise "xor" operator overload
 *
 * @param[in]  lhs        The left hand side
 * @param[in]  rhs        The right hand side
 *
 * @tparam     Enum       The enum
 * @tparam     <unnamed>  Template helper
 *
 * @return     The xor'ed enum
 */
template <typename Enum,
          typename = std::enable_if_t<(enum_operations_enabled<Enum>::value &&
                                       std::is_enum<Enum>::value)>>
inline constexpr Enum operator^(Enum lhs, Enum rhs) {
  using T = typename std::underlying_type_t<Enum>;
  return static_cast<Enum>(static_cast<T>(lhs) ^ static_cast<T>(rhs));
}

/**
 * @brief      Bit-wise "xor" assignment operator overload
 *
 * @param[in]  lhs        The left hand side
 * @param[in]  rhs        The right hand side
 *
 * @tparam     Enum       The enum
 * @tparam     <unnamed>  Template helper
 *
 * @return     A reference to the xor'ed lhs enum
 */
template <typename Enum,
          typename = std::enable_if_t<(enum_operations_enabled<Enum>::value &&
                                       std::is_enum<Enum>::value)>>
inline constexpr Enum& operator^=(Enum lhs, Enum rhs) {
  using T = typename std::underlying_type_t<Enum>;
  lhs     = static_cast<Enum>(static_cast<T>(lhs) ^ static_cast<T>(rhs));
  return lhs;
}

/**
 * @brief      Bit-wise "not" operator overload
 *
 * @param[in]  e          The enum
 *
 * @tparam     Enum       The enum
 * @tparam     <unnamed>  Template helper
 *
 * @return     The not'ed enum
 */
template <typename Enum,
          typename = std::enable_if_t<(enum_operations_enabled<Enum>::value &&
                                       std::is_enum<Enum>::value)>>
inline constexpr Enum& operator~(Enum e) {
  using T = typename std::underlying_type_t<Enum>;
  return static_cast<Enum>(~static_cast<T>(e));
}

};  // namespace exot::utilities
