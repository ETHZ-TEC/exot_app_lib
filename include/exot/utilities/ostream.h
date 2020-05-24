/**
 * @file utilities/ostream.h
 * @author     Bruno Klopott
 * @brief      Output stream overloads for writing complex data structures:
 *             iterables and tuples.
 *
 * @todo       Provide fmt-specific template specifications of the contained
 *             ostream-overloads for performance gains.
 */

#pragma once

#include <chrono>       // for duration
#include <type_traits>  // for enable_if etc.

#include <fmt/format.h>

#include <exot/utilities/formatting.h>  // for Separators
#include <exot/utilities/helpers.h>     // for const_for

/**
 * These have to be registered in the global namespace in order to benefit from
 * fmtlib's ostream formatting.
 */
namespace std {

/**
 * @brief      Outputs iterable types to std::basic_ostream
 *
 * @param      ostream      The output stream
 * @param[in]  range        The iterable range
 *
 * @tparam     T            Type of the iterable range
 * @tparam     CharT        Character type used by basic_ostream
 * @tparam     CharTraitsT  Character traits used by basic_ostream
 *
 * @return     The output stream
 */
template <typename T, typename CharT, typename CharTraitsT>
typename std::enable_if<exot::utilities::is_iterable<T>::value &&
                            !exot::utilities::is_const_iterable<T>::value,
                        std::basic_ostream<CharT, CharTraitsT>&>::type&
operator<<(std::basic_ostream<CharT, CharTraitsT>& ostream, const T& range) {
  exot::utilities::details::Separators<CharT, T> formatting_chars;

  if (formatting_chars.prefix != '\0') ostream << formatting_chars.prefix;

  for (typename T::iterator it = range.begin(); it != range.end(); ++it) {
    if (it == range.begin()) {
      if (std::is_floating_point_v<typename T::value_type>) {
        ostream << fmt::format("{:.4f}", *it);
      } else {
        ostream << *it;
      }
    } else {
      ostream << formatting_chars.delimeter;
      if (std::is_floating_point_v<typename T::value_type>) {
        ostream << fmt::format("{:.4f}", *it);
      } else {
        ostream << *it;
      }
    }
  }
  if (formatting_chars.postfix != '\0') ostream << formatting_chars.postfix;
  return ostream;
};

/**
 * @brief      Outputs const iterable types to std::basic_ostream
 * @details    If the range type is const iterable, prefer this overload.
 *
 * @param      ostream      The output stream
 * @param[in]  range        The const iterable range
 *
 * @tparam     T            Type of the const iterable range
 * @tparam     CharT        Character type used by basic_ostream
 * @tparam     CharTraitsT  Character traits used by basic_ostream
 *
 * @return     The output stream
 */
template <typename T, typename CharT, typename CharTraitsT>
typename std::enable_if<exot::utilities::is_const_iterable<T>::value,
                        std::basic_ostream<CharT, CharTraitsT>&>::type&
operator<<(std::basic_ostream<CharT, CharTraitsT>& ostream, const T& range) {
  exot::utilities::details::Separators<CharT, T> formatting_chars;

  if (formatting_chars.prefix != '\0') ostream << formatting_chars.prefix;

  for (typename T::const_iterator it = range.begin(); it != range.end(); ++it) {
    if (it == range.begin()) {
      if (std::is_floating_point_v<typename T::value_type>) {
        ostream << fmt::format("{:.4f}", *it);
      } else {
        ostream << *it;
      }
    } else {
      ostream << formatting_chars.delimeter;
      if (std::is_floating_point_v<typename T::value_type>) {
        ostream << fmt::format("{:.4f}", *it);
      } else {
        ostream << *it;
      }
    }
  }
  if (formatting_chars.postfix != '\0') ostream << formatting_chars.postfix;
  return ostream;
};

/**
 * @brief      Outputs tuples to std::basic_ostream
 * @details    Uses a compile time for loop to iterate over tuple elements.
 *
 * @param      ostream      The output stream
 * @param[in]  tuple        The tuple
 *
 * @tparam     T            Type of the tuple
 * @tparam     CharT        Character type used by basic_ostream
 * @tparam     CharTraitsT  Character traits used by basic_ostream
 *
 * @return     The output stream
 */
template <typename T, typename CharT, typename CharTraitsT>
typename std::enable_if<exot::utilities::is_tuple<T>::value,
                        std::basic_ostream<CharT, CharTraitsT>>::type&
operator<<(std::basic_ostream<CharT, CharTraitsT>& ostream, const T& tuple) {
  exot::utilities::details::Separators<CharT, T> formatting_chars;

  if (formatting_chars.prefix != '\0') ostream << formatting_chars.prefix;

  /* `const_for` generates a compile-time const index sequence which can be used
   * to access tuple elements via std::get
   */
  exot::utilities::const_for<0, std::tuple_size_v<T>>([&](auto I) {
    if constexpr (I != 0) { ostream << formatting_chars.delimeter; }
    ostream << std::get<I>(tuple);
  });

  if (formatting_chars.postfix != '\0') ostream << formatting_chars.postfix;
  return ostream;
};

/**
 * @brief      Outputs durations to std::basic_ostream
 * @details    Simply calls the `count()` method of the duration object.
 *
 * @param      ostream      The output stream
 * @param[in]  duration     The chrono duration object
 *
 * @tparam     CharT        Character type used by basic_ostream
 * @tparam     CharTraitsT  Character traits used by basic_ostream
 * @tparam     Rep          Underlying type used in the chrono duration
 * @tparam     Period       Ratio used in the chrono duration (e.g. ns)
 *
 * @return     The output stream
 */
template <typename CharT, typename CharTraitsT, typename Rep, typename Period>
std::basic_ostream<CharT, CharTraitsT>& operator<<(
    std::basic_ostream<CharT, CharTraitsT>& ostream,
    const std::chrono::duration<Rep, Period>& duration) {
  ostream << duration.count();
  return ostream;
};

}  // namespace std
