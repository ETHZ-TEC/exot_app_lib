/**
 * @file utilities/literals.h
 * @author     Bruno Klopott
 * @brief      User defined literals.
 */

#pragma once

namespace exot::utilities {

inline namespace literals {

/**
 * @brief      Converts a kibibytes value to bytes
 *
 * @param[in]  v     The value in kibibytes
 *
 * @return     The value in bytes
 */
constexpr inline unsigned long long operator"" _KiB(unsigned long long v) {
  return v * 1024;
}

/**
 * @brief      Converts a mebibytes value to bytes
 *
 * @param[in]  v     The value in mebibytes
 *
 * @return     The value in bytes
 */
constexpr inline unsigned long long operator"" _MiB(unsigned long long v) {
  return v * 1024 * 1024;
}

/**
 * @brief      Converts a gibibytes value to bytes
 *
 * @param[in]  v     The value in gibibytes
 *
 * @return     The value in bytes
 */
constexpr inline unsigned long long operator"" _GiB(unsigned long long v) {
  return v * 1024 * 1024 * 1024;
}

}  // namespace literals

}  // namespace exot::utilities
