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
 * @file utilities/istream.h
 * @author     Bruno Klopott
 * @brief      Input stream overloads for reading tuples and other data types.
 */

#pragma once

#include <array>     // for std::array
#include <chrono>    // for duration
#include <istream>   // for istream
#include <iterator>  // for istream_iterator, back_inserter
#include <sstream>   // for istringstream
#include <tuple>     // for tuple, tuple_cat

#include <exot/utilities/helpers.h>
#include <exot/utilities/types.h>

namespace exot::utilities {

/**
 * @brief      Input stream operator overload for reading durations
 *
 * @param      stream       The input stream
 * @param      duration     The duration object
 *
 * @tparam     CharT        Character type used by the stream
 * @tparam     CharTraitsT  Character traits used by the stream
 * @tparam     Rep          The underlying type used by the duration
 * @tparam     Period       The underlying period type used by the duration
 *
 * @return     The duration with assigned value
 */
template <typename CharT, typename CharTraitsT, class Rep, class Period>
std::istream& operator>>(std::basic_istream<CharT, CharTraitsT>& stream,
                         std::chrono::duration<Rep, Period>& duration) {
  Rep tmp;
  stream >> tmp;
  duration = std::chrono::duration<Rep, Period>(tmp);
  return stream;
}

/**
 * @brief      Reads a tuple from an istream
 *
 * @param      stream       The input stream
 *
 * @tparam     CharT        Character type used by the stream
 * @tparam     CharTraitsT  Character traits used by the stream
 * @tparam     Current      The current tuple element type
 * @tparam     Rest         The types of the other tuple elements
 *
 * @return     A tuple with read values
 */
template <typename CharT, typename CharTraitsT, class Current, class... Rest>
std::tuple<Current, Rest...> read_tuple(
    std::basic_istream<CharT, CharTraitsT>& stream) {
  Current current;
  stream >> current;

  /* Check if there are no more objects to be read, the current being the last.
   * Such solution saves us the trouble creating two template functions for base
   * and recursive case. */
  if constexpr (sizeof...(Rest) == 0) {
    /* The bracket notation is the new uniform initialisation. */
    return std::tuple{current};
  } else {
    /* `tuple_cat` combines tuple elements in a larger tuple, the result is not
     * a tuple of tuples, but rather a tuple holding the types of all individual
     * tuple arguments.
     *
     * The next call to `read_tuple` will receive the remaining tuple types, as
     * long as there are any remaining. */
    return std::tuple_cat(std::tuple{current},
                          read_tuple<CharT, CharTraitsT, Rest...>(stream));
  }
}

namespace details {

/**
 * @brief      Reads a token from an input stream into a input stringstream
 *
 * @param      stream       The input stream
 *
 * @tparam     CharT        Character type used by the stream
 * @tparam     CharTraitsT  Character traits used by the stream
 *
 * @return     A single token as a stringstream
 */
template <typename CharT, typename CharTraitsT>
inline std::basic_istringstream<CharT> read_into_stringstream(
    std::basic_istream<CharT, CharTraitsT>& stream) {
  std::basic_string<CharT> token;
  stream >> token;
  return std::move(std::basic_istringstream<CharT>{token});
}

}  // namespace details

}  // namespace exot::utilities

/**
 * These have to be accessible from the global namespace
 */
namespace std {

/**
 * @brief      Input stream operator overload for reading tuples
 * @todo The function may not work when the tuple holds iterable types
 *
 * @param      stream       The stream
 * @param      tuple        The tuple
 *
 * @tparam     CharT        Character type used by the stream
 * @tparam     CharTraitsT  Character traits used by the stream
 * @tparam     Ts           Types held by the tuple
 *
 * @return     The stream
 */
template <typename CharT, typename CharTraitsT, class... Ts>
std::basic_istream<CharT, CharTraitsT>& operator>>(
    std::basic_istream<CharT, CharTraitsT>& stream, std::tuple<Ts...>& tuple) {
  tuple = exot::utilities::read_tuple<CharT, CharTraitsT, Ts...>(stream);
  return stream;
}

/**
 * @brief      Input stream operator overload for reading into appendable
 *             containers. The container has to have a `push_back()` function.
 *
 * @param      istream      The input stream
 * @param      container    The container
 *
 * @tparam     T            The type of the container
 * @tparam     CharT        Character type used by the stream
 * @tparam     CharTraitsT  Character traits used by the stream
 *
 * @return     The stream
 */
template <typename T, typename CharT, typename CharTraitsT>
typename std::enable_if<(exot::utilities::is_iterable<T>::value &&
                         exot::utilities::has_push_back<T>::value),
                        std::basic_istream<CharT, CharTraitsT>&>::type&
operator>>(std::basic_istream<CharT, CharTraitsT>& istream, T& container) {
  using value_type  = typename T::value_type;
  using string_type = std::basic_string<CharT>;
  using stream_type = std::basic_istringstream<CharT>;

  static constexpr auto delimiter = CharT{','};

  auto stream = exot::utilities::details::read_into_stringstream(istream);
  for (auto [token, input] = std::make_tuple(value_type{}, string_type{});  //
       std::getline(stream, input, delimiter);) {
    stream_type{input} >> token;
    container.push_back(token);
  }

  return istream;
};

/**
 * @brief      Input stream operator overload for reading into fixed-size
 *             arrays.
 *
 * @param      istream      The input stream
 * @param      container    The array
 *
 * @tparam     T            The value type stored in the array
 * @tparam     N            The size of the array
 * @tparam     CharT        Character type used by the stream
 * @tparam     CharTraitsT  Character traits used by the stream
 *
 * @return     The stream
 */
template <typename T, size_t N, typename CharT, typename CharTraitsT>
std::basic_istream<CharT, CharTraitsT>& operator>>(
    std::basic_istream<CharT, CharTraitsT>& istream,
    std::array<T, N>& container) {
  using string_type = std::basic_string<CharT>;
  using stream_type = std::basic_istringstream<CharT>;

  static constexpr auto delimiter = CharT{','};

  auto stream = exot::utilities::details::read_into_stringstream(istream);
  for (auto [i, input] = std::make_tuple(size_t{0}, string_type{});
       std::getline(stream, input, delimiter) && i < N; ++i) {
    stream_type{input} >> container.at(i);
  }

  return istream;
};

}  // namespace std
