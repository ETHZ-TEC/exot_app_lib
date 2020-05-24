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
