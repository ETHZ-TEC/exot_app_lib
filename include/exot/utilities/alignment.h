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
