/**
 * @file utilities/bits.h
 * @author     Bruno Klopott
 * @brief      Type-safe bit manipulation functions.
 */

#pragma once

#include <cerrno>       // for setting errno and macros
#include <limits>       // for numeric_limits
#include <type_traits>  // for enable_if, is_integral_v, is_same_v

#include <fmt/format.h>

#include <exot/utilities/types.h>

namespace exot::utilities {

/**
 * Some of the bit manipulation functions use the compiler's builtins, which
 * should be available for both GCC and LLVM compilers.
 */

/**
 * @brief      Makes a bitmask
 * @details    The function sets bits from lower_bit to upper_bit, e.g. a call
 *             `make_bitmask(2L, 5ULL)` will produce a number `0b11100`. Thanks
 *             to templating, the function should work for many types without
 *             implicit casting, and provides overflow checks for the shift
 *             operations.
 *
 * @param[in]  lower_bit  The lower set bit of the bitmask
 * @param[in]  upper_bit  The upper set bit of the bitmask
 *
 * @tparam     T1         Type of lower_bit
 * @tparam     T2         Type of upper_bit
 * @tparam     <unnamed>  Template helper to enable the function only for
 *                        integral types
 *
 * @return     A bitmask with bits set from lower_bit to upper_bit
 */
template <typename T1, typename T2,
          typename = std::enable_if_t<are_unsigned_integral<T1, T2>>>
inline constexpr larger_of_t<T1, T2> make_bitmask(T1 lower_bit, T2 upper_bit) {
  if (lower_bit > upper_bit) {
    throw std::logic_error("lower bitmask bit > upper bitmask bit");
  }

  if ((lower_bit > std::numeric_limits<T1>::digits) ||
      (upper_bit > std::numeric_limits<T2>::digits))
    throw std::overflow_error(fmt::format(
        "\n\tResulf of shift operation will have undefined value due to"
        "\n\tshifting by value greater or equal to the width of the "
        "type.\n\t{}\n",
        __PRETTY_FUNCTION__));

  /* Performing a shift (2 << by) is the same as doing (1 << (by + 1)). */
  auto upper_mask = (T2{2} << upper_bit) - 1;
  auto lower_mask = (T1{1} << lower_bit) - 1;

  /* Merge lower and upper masks via xor operation. */
  return upper_mask ^ lower_mask;
}

/**
 * @brief      Extracts a value by applying a bitmask
 * @details    The function applies the bitmask, and shifts the value by the
 *             number of trailing zeros in the bitmask. For example, by calling
 *             `extract_with_bitmask(0b11011001, 0b111100)`, we obtain the
 *             result `0b1011`, i.e. `0b11011001 & 0b00111100`, which is
 *             `0b00101100`, then shifted by 2, to produce `0b1011`.
 *
 * @param[in]  value      The value
 * @param[in]  bitmask    The bitmask
 *
 * @tparam     T1         The type of the value
 * @tparam     T2         The type of the bitmask
 * @tparam     <unnamed>  Template helper to enable the function only for
 *                        integral types
 *
 * @return     The extracted value, i.e. the shifted bitmasked original value
 */
template <typename T1, typename T2,
          typename = std::enable_if_t<are_unsigned_integral<T1, T2>>>
inline constexpr T1 extract_with_bitmask(T1 value, T2 bitmask) {
  /* Apply a bitmask and shift by the amount of the trailing zeros in the
   * bitmask. */
  if constexpr (are_same_size<T2, int>) {
    return (value & bitmask) >> __builtin_ctz(bitmask);
  } else if constexpr (are_same_size<T2, long> &&
                       !are_same_size<T2, long long>) {
    return (value & bitmask) >> __builtin_ctzl(bitmask);
  } else if constexpr (are_same_size<T2, long long>) {
    return (value & bitmask) >> __builtin_ctzll(bitmask);
  }
}

template <typename T1, typename T2, typename T3,
          typename = std::enable_if_t<are_unsigned_integral<T1, T2, T3>>>
inline constexpr T1 extract_bit_range(T1 value, T2 lower_bit, T3 upper_bit) {
  return (value & make_bitmask(lower_bit, upper_bit)) >> lower_bit;
}

template <typename T, typename B,
          typename = std::enable_if_t<are_unsigned_integral<T, B>>>
inline constexpr void set_bit(T& value, B bit) {
  value |= (T{1} << bit);
}

template <typename T, typename... Bs,
          typename = std::enable_if_t<are_unsigned_integral<T, Bs...>>>
inline constexpr void set_bits(T& value, Bs... bits) {
  (set_bit(value, bits), ...);
}

template <typename T1, typename T2, typename T3,
          typename = std::enable_if_t<are_unsigned_integral<T1, T2, T3>>>
inline constexpr void set_bit_range(T1& value, T2 lower_bit, T3 upper_bit) {
  value |= make_bitmask(lower_bit, upper_bit);
}

template <typename T, typename B,
          typename = std::enable_if_t<are_unsigned_integral<T, B>>>
inline constexpr T get_bit(T value, B bit) {
  return (value >> bit) & T{1};
}

template <typename T, typename B,
          typename = std::enable_if_t<are_unsigned_integral<T, B>>>
inline constexpr void clear_bit(T& value, B bit) {
  value &= ~(T{1} << bit);
}

template <typename T, typename... Bs,
          typename = std::enable_if_t<are_unsigned_integral<T, Bs...>>>
inline constexpr void clear_bits(T& value, Bs... bits) {
  (clear_bit(value, bits), ...);
}

template <typename T1, typename T2, typename T3,
          typename = std::enable_if_t<are_unsigned_integral<T1, T2, T3>>>
inline constexpr void clear_bit_range(T1& value, T2 lower_bit, T3 upper_bit) {
  value &= ~make_bitmask(lower_bit, upper_bit);
}

template <typename T, typename B,
          typename = std::enable_if_t<are_unsigned_integral<T, B>>>
inline constexpr void toggle_bit(T& value, B bit) {
  value ^= (T{1} << bit);
}

template <typename T, typename... Bs,
          typename = std::enable_if_t<are_unsigned_integral<T, Bs...>>>
inline constexpr void toggle_bits(T& value, Bs... bits) {
  (toggle_bit(value, bits), ...);
}

template <typename T1, typename T2, typename T3,
          typename = std::enable_if_t<are_unsigned_integral<T1, T2, T3>>>
inline constexpr void toggle_bit_range(T1& value, T2 lower_bit, T3 upper_bit) {
  value ^= make_bitmask(lower_bit, upper_bit);
}

template <typename T, typename B,
          typename = std::enable_if_t<are_unsigned_integral<T, B>>>
inline bool test_bit(T value, B bit) {
  return value & (T{1} << bit);
}

template <typename T, typename... Bs,
          typename = std::enable_if_t<are_unsigned_integral<T, Bs...>>>
inline bool test_bits(T value, Bs... bits) {
  return (test_bit(value, bits) && ...);
}

template <typename T1, typename T2, typename T3,
          typename = std::enable_if_t<are_unsigned_integral<T1, T2, T3>>>
inline constexpr bool test_bit_range(T1 value, T2 lower_bit, T3 upper_bit) {
  auto mask      = make_bitmask(lower_bit, upper_bit);
  auto extracted = extract_with_bitmask(value, mask);
  return extracted != decltype(extracted){0};
}

/**
 * @brief      Return the hamming weight of a number, the count of bits set to 1
 *
 * @param[in]  value      The evaluated value
 *
 * @tparam     T          The type of the value
 * @tparam     <unnamed>  Template helper
 *
 * @return     The count of individual bits set to 1
 */
template <typename T, typename = std::enable_if_t<are_unsigned_integral<T>>>
inline constexpr int hamming_weight(T value) {
  if constexpr (are_same_size<T, int>) {
    return __builtin_popcount(value);
  } else if constexpr (are_same_size<T, long> &&
                       !are_same_size<long, long long>) {
    return __builtin_popcountl(value);
  } else if constexpr (are_same_size<T, long long>) {
    return __builtin_popcountll(value);
  }
}

/**
 * @brief      Return the parity of the value (evenness/oddness of an integer)
 *
 * @param[in]  value      The evaluated value
 *
 * @tparam     T          The type of the value
 * @tparam     <unnamed>  Template helper
 *
 * @return     The parity of the value
 */
template <typename T, typename = std::enable_if_t<are_unsigned_integral<T>>>
inline constexpr int parity(T value) {
  if constexpr (are_same_size<T, int>) {
    return __builtin_parity(value);
  } else if constexpr (are_same_size<T, long> &&
                       !are_same_size<long, long long>) {
    return __builtin_parityl(value);
  } else if constexpr (are_same_size<T, long long>) {
    return __builtin_parityll(value);
  }
}

/**
 * @brief      Gets the number of trailing zero bits in an unsigned integer
 * @note       Takes care of the case when 0 is given, for which BSR or BSF
 *             operations are undefined. In such case the number of digits in
 *             the type T is returned.
 *
 * @param[in]  value      The evaluated value
 *
 * @tparam     T          The type of the value
 * @tparam     <unnamed>  Template helper
 *
 * @return     The number of trailing bits in the value
 */
template <typename T, typename = std::enable_if_t<are_unsigned_integral<T>>>
inline constexpr int trailing_zero_bits(T value) {
  if constexpr (are_same_size<T, int>) {
    return value ? __builtin_ctz(value) : std::numeric_limits<T>::digits;
  } else if constexpr (are_same_size<T, long> &&
                       !are_same_size<long, long long>) {
    return value ? __builtin_ctzl(value) : std::numeric_limits<T>::digits;
  } else if constexpr (are_same_size<T, long long>) {
    return value ? __builtin_ctzll(value) : std::numeric_limits<T>::digits;
  }
}

/**
 * @brief      Gets the number of leading zero bits in an unsigned integer
 * @note       Takes care of the case when 0 is given, for which BSR or BSF
 *             operations are undefined. In such case the number of digits in
 *             the type T is returned.
 *
 * @param[in]  value      The evaluated value
 *
 * @tparam     T          The type of the value
 * @tparam     <unnamed>  Template helper
 *
 * @return     The number of leading bits in the value
 */
template <typename T, typename = std::enable_if_t<are_unsigned_integral<T>>>
inline constexpr int leading_zero_bits(T value) {
  if constexpr (are_same_size<T, int>) {
    return value ? __builtin_clz(value) : std::numeric_limits<T>::digits;
  } else if constexpr (are_same_size<T, long> &&
                       !are_same_size<long, long long>) {
    return value ? __builtin_clzl(value) : std::numeric_limits<T>::digits;
  } else if constexpr (are_same_size<T, long long>) {
    return value ? __builtin_clzll(value) : std::numeric_limits<T>::digits;
  }
}

/**
 * @brief      Gets the position of the next set bit in an unsigned integer
 * @note       If the 'bit' value is greater than the number of digits in the
 *             type T, the function returns the latter, and sets errno to EDOM.
 * @details    The function can be use to loop through set bits in a number,
 *             like so:
 *
 *             ```cpp
 *             for (auto i = next_set_bit(value, 0);
                    i < std::numeric_limits<decltype(value)>::digits;
                    i = next_set_bit(value, i + 1));
 *              ```
 *
 * @param[in]  value      The value
 * @param[in]  bit        The bit from which the testing is performed
 *
 * @tparam     T          The type of the value
 * @tparam     <unnamed>  Template helper
 *
 * @return     The bit position of the next set bit
 */
template <typename T, typename = std::enable_if_t<are_unsigned_integral<T>>>
inline constexpr std::size_t next_set_bit(T value, std::size_t bit) {
  /* T{1} << bit - 1 clears the lower 'bit' bits, see clear_bit above.
   *
   * We also need to account for the case when the set bit is greater than the
   * number of digits in the type T, because the operation T{1} << bit would be
   * incorrect. Here we return the number of digits, and also set the errno
   * flag. */
  if (bit >= std::numeric_limits<T>::digits) {
    errno = EDOM;
    return std::numeric_limits<T>::digits;
  }

  return trailing_zero_bits(value & ~((T{1} << bit) - 1));
}

/**
 * @brief      Executes a callable with indeces of set bits in a value
 *
 * @param[in]  value      The value
 * @param[in]  callable   The callable, must be invocable as void(size_t)
 *
 * @tparam     T          The type of the value
 * @tparam     F          The callable type
 * @tparam     <unnamed>  Template helper
 */
template <typename T, typename F,
          typename = std::enable_if_t<
              (are_unsigned_integral<T> &&
               std::is_invocable_r<void, F, std::size_t>::value)>>
inline constexpr void loop_through_set_bits(T value, F&& callable) {
  for (auto i = exot::utilities::next_set_bit(value, 0);
       i < std::numeric_limits<T>::digits - 1;
       i = exot::utilities::next_set_bit(value, i + 1)) {
    callable(i);
  }
}

}  // namespace exot::utilities
