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
 * @file utilities/helpers.h
 * @author     Bruno Klopott
 * @brief      Helper templates, among others for compile-time code generation,
 *             tuple access, and statistics.
 */

#pragma once

#include <algorithm>  // for generate
#include <array>      // for fixed-size arrays
#include <cmath>      // for std::sqrt
#include <numeric>    // for inner_product, accumulate,...
#include <tuple>      // for packing into and operating on tuples
#include <utility>    // for integral_constant, etc.
#include <vector>     // for variable-size arrays

#include <exot/utilities/types.h>

namespace exot::utilities {

namespace details {

/**
 * @brief      Helper template used by `const_for`
 * @details    The helper uses pack expansion to construct a compile-time
 *             sequence of `std::size_t` integers, which are then incrementally
 *             provided to a callable object, also by means of parameter pack
 *             expansion.
 *
 *             `std::size_t` is used because it is commonly used for array
 *             indexing and loop counting. Moreover, this allows the use of the
 *             `sizeof` operator, which return type is `size_t`.
 *
 * @param[in]  func       A callable object (e.g. lambda, functor)
 * @param[in]  <unnamed>  A compile-time sequence of `std::size_t` integers,
 *                        used in template pack expansion
 *
 * @tparam     Begin      First sequence number (similar to the first parameter
 *                        in a `for` loop declaration)
 * @tparam     Callable   A callable type
 * @tparam     I          Sequence of `std::size_t` types
 */
template <std::size_t Begin, typename Callable, std::size_t... I>
constexpr void const_for_impl(Callable&& func, std::index_sequence<I...>) {
  /* std::integral_constant<T,V> makes a static constexpr type T with value V,
   * so the function `func` below is provided with a constexpr `std::size_t`
   * value (Begin + I), which can later be used in templates and static
   * contexts.
   *
   * The pack expansion over `...` produces code:
   * @code{c++}
   * (func(Begin), func(Begin+1), func(Begin+2), ...);
   * @endcode
   */
  (func(std::integral_constant<std::size_t, Begin + I>{}), ...);
};

/**
 * @brief      Helper template used by `for_each`
 * @details    The helper uses a compile-time sequence of integers and pack
 *             expansion to access tuple elements via `std::get`.
 *
 * @param[in]  tuple      A tuple
 * @param[in]  func       A callable object
 * @param[in]  <unnamed>  A compile-time sequence of `std::size_t` integers,
 *                        used in template pack expansion
 *
 * @tparam     Callable   A callable type
 * @tparam     Tuple      The type of the tuple
 * @tparam     I          Sequence of `std::size_t` types
 */
template <typename Callable, typename Tuple, std::size_t... I>
constexpr void for_each_impl(Tuple&& tuple, Callable&& func,
                             std::index_sequence<I...>) {
  /* The pack expansion produces code: `(func(get<0>(tuple)),
   * func(get<1>(tuple)), ...);`. */
  (func(std::get<I>(tuple)), ...);
};

/**
 * @brief Makes a tuple out of values of tuples at a specific index position
 *
 * @tparam I The index
 * @tparam Ts The tuples' types
 * @param ts The tuples
 * @return A tuple of values at position I
 */
template <std::size_t I, typename... Ts>
auto zip_at_idx(Ts&&... ts) {
  return std::make_tuple(std::get<I>(std::forward<Ts>(ts))...);
}

/**
 * @brief Zips a number of tuples
 *
 * @tparam Ts The tuples' types
 * @tparam I The index sequence
 * @param ts The tuples
 * @return The zipped tupples, a tuple of tuples
 */
template <typename... Ts, std::size_t... I>
auto zip_impl(Ts&&... ts, std::index_sequence<I...>) {
  return std::make_tuple(zip_at_idx<I>(std::forward<Ts>(ts)...)...);
}

/**
 * @brief Makes a tuple with forwarded values of tuples at specific index
 *
 * @tparam I The index
 * @tparam Ts The tuples' types
 * @param ts The tuples
 * @return A tuple with forwarded values
 */
template <std::size_t I, typename... Ts>
auto zip_fwd_at_idx(Ts&&... ts) {
  return std::forward_as_tuple(std::get<I>(std::forward<Ts>(ts))...);
}

/**
 * @brief Zips a number of tuples by forwarding values
 *
 * @tparam Ts The tuples' types
 * @tparam I The index sequence
 * @param ts The tuples
 * @return The zipped tupples, a tuple of tuples
 */
template <typename... Ts, std::size_t... I>
auto zip_fwd_impl(Ts&&... ts, std::index_sequence<I...>) {
  return std::forward_as_tuple(zip_fwd_at_idx<I>(std::forward<Ts>(ts)...)...);
}

template <std::size_t I, typename... Ts, std::size_t... Is>
auto slice_tuple(std::tuple<Ts...>&& _t, std::index_sequence<Is...>) {
  return std::make_tuple(std::get<I + Is>(_t)...);
}

template <std::size_t I, typename... Ts, std::size_t... Is>
auto slice_fwd_tuple(std::tuple<Ts...>&& _t, std::index_sequence<Is...>) {
  return std::forward_as_tuple(std::get<I + Is>(_t)...);
}

}  // namespace details

/**
 * @brief      A compile-time for-loop
 * @details    Uses the helper above to generate a sequence of `std::size_t`
 *             integers from Begin to End, invokes the callable object with the
 *             constexpr index, e.g. `std::integral_constant<unsigned long,
 *             0ul>` on a 64-bit architecture. If a lambda is provided, the call
 *             will be to an anonymous class' `operator() const`. Functions
 *             provided to the static for-loop should also be constexpr.
 *
 * @param[in]  func  A callable object
 *
 * @tparam     Begin      First sequence number (similar to the first parameter
 *                        in a `for` loop declaration)
 * @tparam     End        Last sequence number (similar to the second parameter
 *                        in a `for` loop declaration, increment is implied)
 * @tparam     Callable   A callable type
 */
template <std::size_t Begin, std::size_t End, typename Callable>
constexpr void const_for(Callable&& func) {
  /* `std::make_index_sequence<N>` creates `std::size_t` sequence from 0 to N-1,
   * therefore the helper generates a sequence from Begin to (End-Begin-1). */
  details::const_for_impl<Begin>(std::forward<Callable>(func),
                                 std::make_index_sequence<End - Begin>{});
};

/**
 * @brief      Apply a function to each tuple element
 *
 * @param[in]  tuple      The tuple
 * @param[in]  func       A callable object
 *
 * @tparam     Callable   A callable type
 * @tparam     Ts         Individual tuple types
 */
template <typename Callable, typename... Ts>
constexpr void for_each(std::tuple<Ts...>&& tuple, Callable&& func) {
  details::for_each_impl(std::forward<decltype(tuple)>(tuple),
                         std::forward<Callable>(func),
                         std::index_sequence_for<Ts...>{});
};

/* If std::apply is available in the `<tuple>` header. */
#ifdef __cpp_lib_apply

/**
 * @brief      Wrapper for std::apply
 * @details    Applies a non-returning function to each tuple element, using
 *             pack expansion. The tuple elements are processed in order.
 *
 * @param[in]  f          The function to apply
 * @param[in]  tuple      The tuple to operate on
 *
 * @tparam     Func       A callable type
 * @tparam     Tuple      A tuple type
 */
template <typename Func, typename Tuple>
constexpr void apply(Func&& f, Tuple&& tuple) {
  /* Produces same behaviour as `for_each`. */
  std::apply([&f](auto&... x) { (..., f(x)); }, std::forward<Tuple>(tuple));
};

template <typename Head, typename... Tail>
auto tail(std::tuple<Head, Tail...>&& value) {
  return std::apply(
      [](auto&& head, auto&&... tail) {
        return std::forward_as_tuple(tail...);
      },
      value);
}

#else

template <typename... Ts>
auto tail(std::tuple<Ts...>&& value) {
  return details::slice_fwd_tuple<1ull>(
      std::forward<decltype(value)>(value),
      std::make_index_sequence<sizeof...(Ts) - 1ull>());
}

#endif

/**
 * @brief Zips multiple tuples together
 *
 * @tparam T The first tuple type
 * @tparam Ts The other tuples' types
 * @param t The first tuple
 * @param ts The other tuples
 * @return The zipped tuples
 */
template <typename T, typename... Ts>
auto zip(T&& t, Ts&&... ts) {
  static_assert((... && (std::tuple_size_v<std::decay_t<T>> ==
                         std::tuple_size_v<std::decay_t<Ts>>)),
                "must be of same size");

  return details::zip_impl<T, Ts...>(
      std::forward<T>(t), std::forward<Ts>(ts)...,
      std::make_index_sequence<std::tuple_size_v<std::decay_t<T>>>());
}

/**
 * @brief Zips multiple tuples together by forwarding values
 *
 * @tparam T The first tuple type
 * @tparam Ts The other tuples' types
 * @param t The first tuple
 * @param ts The other tuples
 * @return The zipped tuples
 */
template <typename T, typename... Ts>
auto zip_fwd(T&& t, Ts&&... ts) {
  static_assert((... && (std::tuple_size_v<std::decay_t<T>> ==
                         std::tuple_size_v<std::decay_t<Ts>>)),
                "must be of same size");

  return details::zip_fwd_impl(
      std::forward<T>(t), std::forward<Ts>(ts)...,
      std::make_index_sequence<std::tuple_size_v<std::decay_t<T>>>());
}

/**
 * @brief      Calculates the mean and standard deviation of values in a
 * container
 *
 * @param[in]  input      The input container
 *
 * @tparam     T          The value type
 * @tparam     <unnamed>  Template helper
 *
 * @return     The mean and deviation as an array of doubles
 */
template <typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
auto mean_and_deviation(const std::vector<T>& input) -> std::array<double, 2> {
  auto mean =
      static_cast<double>(std::accumulate(input.begin(), input.end(), (T{0}))) /
      input.size();

  using difference_type = decltype(T{0} - T{0});

  std::vector<difference_type> deviations(input.size());

  std::transform(input.begin(), input.end(), deviations.begin(),
                 [mean](auto in) { return (in - mean); });

  /* The accumulator {a} in inner_product calculates: a += *iterator1 *
   * *iterator2, starts with 0.0. Computes the squared sum(x - mean)(x - mean)
   * in variance computation. */
  auto squared_sum = std::inner_product(deviations.begin(), deviations.end(),
                                        deviations.begin(), difference_type{0});

  double standard_deviation = std::sqrt((double)squared_sum / input.size());

  return {mean, standard_deviation};
}

/**
 * @brief      Gets a slice of an interable container
 *
 * @param[in]  container  The container
 * @param[in]  lower      The lower index
 * @param[in]  upper      The upper index
 *
 * @tparam     T          The type of the container
 * @tparam     <unnamed>  Template helper
 *
 * @return     The slice, reversed if lower < upper
 */
template <typename T,
          typename = std::enable_if_t<exot::utilities::is_iterable_v<T>>>
constexpr auto slice(const T& container, std::size_t lower, std::size_t upper)
    -> T {
  if (lower <= upper) {
    return T(std::begin(container) + lower, std::begin(container) + upper);
  } else {
    T dest(std::distance(std::begin(container) + upper,
                         std::begin(container) + lower));
    std::reverse_copy(std::begin(container) + upper,
                      std::begin(container) + lower, std::begin(dest));
    return dest;
  }
}

/**
 * @brief      Gets a percentile of values in a container
 * @note       Only arithmetic value types are supported.
 *
 * @param[in]  container  The container
 * @param[in]  value      The percentile in range (0, 1)
 *
 * @tparam     T          The container type
 * @tparam     <unnamed>  Template helper
 *
 * @return     The percentile value of elements in the container.
 */
template <typename T, typename = std::enable_if_t<
                          (exot::utilities::is_location_accessible<T>::value &&
                           exot::utilities::is_iterable_v<T> &&
                           std::is_arithmetic_v<typename T::value_type>)>>
constexpr auto percentile(T container, double value) -> typename T::value_type {
  if (container.empty())
    throw std::logic_error(
        "Cannot calculate percentiles for an empty container.");

  if ((value >= 1) || (value <= 0))
    throw std::out_of_range("Percentiles must be in range (0,1).");

  auto position = static_cast<std::size_t>(std::max(
      0ul,
      std::min(static_cast<std::size_t>(std::ceil(container.size() * value)),
               container.size() - 1ul)));

  if (!std::is_sorted(std::begin(container), std::end(container)))
    std::sort(std::begin(container), std::end(container));

  return container.at(position);
}

/**
 * @brief      Gets a percentile of values in a container, sorting it in place
 * @note       Only arithmetic value types are supported. The side effect is
 *             that the input container will become sorted.
 *
 * @param[in]  container  The container
 * @param[in]  value      The percentile in range (0, 1)
 *
 * @tparam     T          The container type
 * @tparam     <unnamed>  Template helper
 *
 * @return     The percentile value of elements in the container.
 */
template <typename T, typename = std::enable_if_t<
                          (exot::utilities::is_location_accessible<T>::value &&
                           exot::utilities::is_iterable_v<T> &&
                           std::is_arithmetic_v<typename T::value_type>)>>
constexpr auto percentile_sort_in_place(T& container, double value) ->
    typename T::value_type {
  if (container.empty())
    throw std::logic_error(
        "Cannot calculate percentiles for an empty container.");

  if ((value >= 1) || (value <= 0))
    throw std::out_of_range("Percentiles must be in range (0,1).");

  auto position = static_cast<std::size_t>(std::max(
      0ul,
      std::min(static_cast<std::size_t>(std::ceil(container.size() * value)),
               container.size() - 1ul)));

  if (!std::is_sorted(std::begin(container), std::end(container)))
    std::sort(std::begin(container), std::end(container));

  return container.at(position);
}

/**
 * @brief      Gets a slice of a container in percentile limits
 *
 * @param[in]  container  The container
 * @param[in]  lower      The lower percentile in range (0, 1]
 * @param[in]  upper      The upper percentile in range (0, 1]
 *
 * @tparam     T          The container type
 * @tparam     <unnamed>  Template helper
 *
 * @return     A slice of the container within limits, reversed if lower > upper
 */
template <typename T, typename = std::enable_if_t<
                          (exot::utilities::is_location_accessible<T>::value &&
                           exot::utilities::is_iterable_v<T> &&
                           std::is_arithmetic_v<typename T::value_type>)>>
constexpr auto percentile_range(T container, double lower, double upper) -> T {
  if (container.empty())
    throw std::logic_error(
        "Cannot calculate percentiles for an empty container.");

  if (!std::is_sorted(std::begin(container), std::end(container)))
    std::sort(std::begin(container), std::end(container));

  if ((lower > 1) || (lower <= 0) || (upper > 1) || (upper <= 0))
    throw std::out_of_range("Percentiles must be in range (0,1].");

  auto llim = static_cast<std::size_t>(std::max(
      0ul,
      std::min(static_cast<std::size_t>(std::ceil(container.size() * lower)),
               container.size() - 1ul)));

  auto ulim = static_cast<std::size_t>(std::max(
      0ul,
      std::min(static_cast<std::size_t>(std::ceil(container.size() * upper)),
               container.size() - 1ul)));

  return slice(container, llim, ulim);
}

/**
 * @brief      Determines if power of 2.
 *
 * @param[in]  value      The value
 *
 * @tparam     T          The type of the value (unsigned integral)
 * @tparam     <unnamed>  Template helper
 *
 * @return     True if power of 2, False otherwise.
 */
template <typename T, typename = std::enable_if_t<std::is_integral_v<T> &&
                                                  std::is_unsigned_v<T>>>
constexpr inline bool is_power_of_2(T value) noexcept {
  return (value > 0) && ((value & (value - 1)) == 0);
}

/**
 * @brief      Determines if a value is within a set of other values.
 *
 * @param[in]  t          The value to compare
 * @param[in]  ts         The set of values to compare against
 *
 * @tparam     T          The type of the value to compare
 * @tparam     Ts         The types of the values to compare against
 * @tparam     <unnamed>  Template helper which determines if all decayed types
 *                        are the same
 *
 * @return     True if t is one of ts, False otherwise.
 */
template <typename T, typename... Ts,
          typename = typename std::enable_if<
              (... && is_convertible_d<T, Ts>::value)>::type>
inline constexpr bool is_one_of(T&& t, Ts&&... ts) {
  return (... || (t == ts));
}

/**
 * @brief      Determines if a value is not in a set of other values.
 *
 * @param[in]  t          The value to compare
 * @param[in]  ts         The set of values to compare against
 *
 * @tparam     T          The type of the value to compare
 * @tparam     Ts         The types of the values to compare against
 * @tparam     <unnamed>  Template helper which determines if all decayed types
 *                        are the same
 *
 * @return     True if t isn't one of ts, False otherwise.
 */
template <typename T, typename... Ts,
          typename = typename std::enable_if<
              (... && is_convertible_d<T, Ts>::value)>::type>
inline constexpr bool is_none_of(T&& t, Ts&&... ts) {
  return !is_one_of(std::forward<T>(t), std::forward<Ts>(ts)...);
}

/**
 * @brief      Check if a container contains an element
 * @details    The provided type T must conform to the 'is_iterable' type trait,
 *             the value type is obtained from a member type/alias 'value_type'.
 *
 * @param[in]  v          The value to check
 * @param[in]  container  The container
 *
 * @tparam     T          The type of the container
 * @tparam     <unnamed>  Template helper
 *
 * @return     True if found, false otherwise.
 */
template <typename T, typename = std::enable_if_t<is_iterable_v<T>>>
auto contains(typename T::value_type v, const T& container) -> bool {
  return std::find(std::begin(container), std::end(container), v) !=
         std::end(container);
}

template <typename T, typename... Ts,
          typename = std::enable_if_t<(is_convertible_d<T, Ts>::value && ...) &&
                                      is_comparable_v<T>>>
constexpr bool is_greater(T t, Ts&&... ts) {
  return (... && (t > ts));
}

template <typename T, typename... Ts,
          typename = std::enable_if_t<(is_convertible_d<T, Ts>::value && ...) &&
                                      is_comparable_v<T>>>
constexpr bool is_greater_or_equal(T t, Ts&&... ts) {
  return (... && (t >= ts));
}

template <typename T, typename... Ts,
          typename = std::enable_if_t<(is_convertible_d<T, Ts>::value && ...) &&
                                      is_comparable_v<T>>>
constexpr bool assert_is_greater(T t, Ts&&... ts) {
  return (... && (t > ts))
             ? true
             : throw std::logic_error("First operand is not larger then rest");
}

template <typename T, typename... Ts,
          typename = std::enable_if_t<(is_convertible_d<T, Ts>::value && ...) &&
                                      is_comparable_v<T>>>
constexpr bool assert_is_greater_or_equal(T t, Ts&&... ts) {
  return (... && (t >= ts))
             ? true
             : throw std::logic_error(
                   "First operand is not larger or equal then rest");
}

template <typename T, typename... Ts,
          typename = std::enable_if_t<(is_convertible_d<T, Ts>::value && ...) &&
                                      is_comparable_v<T>>>
constexpr bool is_smaller(T t, Ts&&... ts) {
  return (... && (t < ts));
}

template <typename T, typename... Ts,
          typename = std::enable_if_t<(is_convertible_d<T, Ts>::value && ...) &&
                                      is_comparable_v<T>>>
constexpr bool is_smaller_or_equal(T t, Ts&&... ts) {
  return (... && (t <= ts));
}

template <typename T, typename... Ts,
          typename = std::enable_if_t<(is_convertible_d<T, Ts>::value && ...) &&
                                      is_lessthan_comparable_v<T>>>
constexpr bool assert_is_smaller(T t, Ts&&... ts) {
  return (... && (t < ts))
             ? true
             : throw std::logic_error("First operand is not smaller then rest");
}

template <typename T, typename... Ts,
          typename = std::enable_if_t<(is_convertible_d<T, Ts>::value && ...) &&
                                      is_comparable_v<T>>>
constexpr bool assert_is_smaller_or_equal(T t, Ts&&... ts) {
  return (... && (t <= ts))
             ? true
             : throw std::logic_error(
                   "First operand is not smaller or equal then rest");
}

/* The following helpers should work on all compilers that have been used
 * for the project: GCC and LLVM Clang. */

/**
 * @brief      Helper to prevent the compiler from optimising a variable
 *
 * @param      var   The const reference to the variable
 *
 * @tparam     T     The type of the variable
 */
template <typename T>
inline __attribute__((always_inline)) void do_not_optimise(T const& var) {
  asm volatile("" : : "r,m"(var) : "memory");
}

/**
 * @brief      Helper to prevent the compiler from optimising a variable
 *
 * @param      var   The reference to the variable
 *
 * @tparam     T     The type of the variable
 */
template <typename T>
inline __attribute__((always_inline)) void do_not_optimise(T& var) {
#if defined(__clang__)
  asm volatile("" : "+r,m"(var) : : "memory");
#else
  asm volatile("" : "+m,r"(var) : : "memory");
#endif
}

template <typename... Ts>
inline __attribute__((always_inline)) void do_not_optimise(Ts&&... vars) {
  static_assert((std::is_lvalue_reference<Ts>::value && ...),
                "all vars need to be references (or const references)");
  (..., do_not_optimise(std::forward<Ts>(vars)));
}

/**
 * Alias for US spelling of 'optimise'.
 */
template <typename... Ts>
inline __attribute__((always_inline)) void do_not_optimize(Ts&&... vars) {
  do_not_optimise(std::forward<Ts>(vars)...);
}

/**
 * @brief      Determines if values or function returns are constexpr.
 * @note       Cannot be used on functions with signatures void(...).
 *
 * @param      ts    The values
 *
 * @tparam     Ts    The value types
 *
 * @return     True if all are constexpr, False otherwise.
 */
template <typename... Ts>
constexpr bool is_constexpr(Ts&&... ts) {
  return (noexcept(ts) && ...);
}

}  // namespace exot::utilities
