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
 * @file utilities/types.h
 * @author     Bruno Klopott
 * @brief      Custom type traits.
 */

#pragma once

#include <chrono>       // for chrono types and clocks
#include <iostream>     // for streams
#include <tuple>        // for tuples
#include <type_traits>  // for true_type, false_type, declval, decay_t...
#include <vector>       // for variable-size arrays

/**
 * Forward declarations of TConsumer, TProducer, TProcessor.
 */
namespace exot::framework {
struct TConsumer;
struct TProducer;
struct TProcessor;
}  // namespace exot::framework

namespace exot::utilities {

/**
 * @brief      Is the given type a std::chrono::duration?
 *
 * @tparam     T     The type to check
 */
template <typename T>
struct is_duration  //
    : public std::false_type {};

template <typename Rep, typename Period>
struct is_duration<std::chrono::duration<Rep, Period>>  //
    : public std::true_type {};

template <typename T>
inline constexpr bool is_duration_v = is_duration<T>::value;

/**
 * @brief      Is the given type a std::chrono-like clock?
 *
 * @tparam     T          The type to check
 * @tparam     <unnamed>  Template helper
 */
template <typename T, typename = void>
struct is_clock : std::false_type {};

template <typename T>
struct is_clock<T,
                std::void_t<typename std::decay_t<T>::duration,    //
                            typename std::decay_t<T>::rep,         //
                            typename std::decay_t<T>::period,      //
                            typename std::decay_t<T>::time_point,  //
                            decltype(std::declval<std::decay_t<T>>().now())>>
    : std::true_type {};

template <typename T>
inline constexpr bool is_clock_v = is_clock<T>::value;

/**
 * @brief      Determines if a type is EqualityComparable (named C++ req.)
 *
 * @tparam     T          The type
 * @tparam     <unnamed>  Template helper
 */
template <typename T, typename = void>
struct is_equality_comparable : std::false_type {};

template <typename T>
struct is_equality_comparable<
    T, std::void_t<decltype(std::declval<T&>() == std::declval<T&>())>>
    : std::true_type {};

template <typename T>
inline constexpr bool is_equality_comparable_v =
    is_equality_comparable<T>::value;

/**
 * @brief      Determines if a type is LessThanComparable (named C++ req.)
 *
 * @tparam     T          The type
 * @tparam     <unnamed>  Template helper
 */
template <typename T, typename = void>
struct is_lessthan_comparable : std::false_type {};

template <typename T>
struct is_lessthan_comparable<
    T, std::void_t<decltype(std::declval<T&>() < std::declval<T&>())>>
    : std::true_type {};

template <typename T>
inline constexpr bool is_lessthan_comparable_v =
    is_lessthan_comparable<T>::value;

/**
 * @brief      Determines if a type is EqualityComparable and LessThanComparable
 *
 * @tparam     T          The type
 * @tparam     <unnamed>  Template helper
 */
template <typename T, typename = void>
struct is_comparable : std::false_type {};

template <typename T>
struct is_comparable<
    T, std::void_t<decltype(std::declval<T&>() == std::declval<T&>()),
                   decltype(std::declval<T&>() < std::declval<T&>())>>
    : std::true_type {};

template <typename T>
inline constexpr bool is_comparable_v = is_comparable<T>::value;

/**
 * Helper compile-time contant for checking if integral types are of same size.
 */
template <typename T1, typename T2>
inline constexpr bool are_same_size = (sizeof(T1) == sizeof(T2));

/**
 * @brief      Provides the larger of two types as a member type
 *
 * @tparam     T1    lhs type
 * @tparam     T2    rhs type
 */
template <typename T1, typename T2>
struct larger_of : std::conditional<(sizeof(T1) >= sizeof(T2)), T1, T2> {};

/**
 * Helper alias for larger_of
 */
template <typename T1, typename T2>
using larger_of_t = typename larger_of<T1, T2>::type;

/**
 * @brief      Provides the smaller of two types as a member type
 *
 * @tparam     T1    lhs type
 * @tparam     T2    rhs type
 */
template <typename T1, typename T2>
struct smaller_of : std::conditional<(sizeof(T1) >= sizeof(T2)), T2, T1> {};

/**
 * Helper alias for smaller_of
 */
template <typename T1, typename T2>
using smaller_of_t = typename smaller_of<T1, T2>::type;

/**
 * Helper compile-time contant for checking if all types are integral types
 */
template <typename... Ts>
inline constexpr bool are_integral = (std::is_integral_v<std::decay_t<Ts>> &&
                                      ...);

/**
 * Helper compile-time contant for checking if all types are unsigned types
 */
template <typename... Ts>
inline constexpr bool are_unsigned = (std::is_unsigned_v<std::decay_t<Ts>> &&
                                      ...);

/**
 * Helper compile-time contant for checking if all types are unsigned itegrals
 */
template <typename... Ts>
inline constexpr bool are_unsigned_integral = (are_integral<Ts...> &&
                                               are_unsigned<Ts...>);

/**
 * Alias for a decayed is_same type trait.
 */
template <typename T, typename U>
using is_same_d = typename std::is_same<std::decay_t<T>, std::decay_t<U>>;

/**
 * Alias for a decayed is_same type trait.
 */
template <typename T, typename U>
using is_convertible_d =
    typename std::is_convertible<std::decay_t<T>, std::decay_t<U>>;

/**
 * @brief      Compile-time helper for decayed string-convertible check
 */
template <typename T>
inline constexpr bool is_string_d = is_convertible_d<T, std::string>::value;

/**
 * @brief      Compile-time helper for decayed arithmetic check
 */
template <typename T>
inline constexpr bool is_arithmetic_d =
    std::is_arithmetic<std::decay_t<T>>::value;

/**
 * @brief      Compile-time helper for decayed pointer check
 */
template <typename T>
inline constexpr bool is_pointer_d =
    (std::is_pointer<std::decay_t<T>>::value ||
     std::is_member_pointer<std::decay_t<T>>::value ||
     std::is_null_pointer<std::decay_t<T>>::value);

/**
 * @brief      Type trait for character types
 * @details    Enabled if CharT is a standard or wide character.
 *
 * @tparam     CharT   The character type
 * @tparam     _       Template helper
 */
template <typename CharT, typename _ = void>
struct is_character : std::false_type {};

template <typename CharT>
struct is_character<CharT, std::enable_if_t<(std::is_same_v<CharT, char> ||
                                             std::is_same_v<CharT, wchar_t> ||
                                             std::is_same_v<CharT, char16_t> ||
                                             std::is_same_v<CharT, char32_t>)>>
    : std::true_type {};

/**
 * @brief      Compile time value for the `is_character` structure.
 */
template <typename CharT>
inline constexpr bool is_character_v = is_character<CharT>::value;

/**
 * @brief      Casts a type (e.g. scoped enum class) to its underlying type.
 * @details    This function may come useful when used in conjunction with a
 *             typed enum, e.g. `enum class X : int {};`, for which the
 *             operation `to_underlying_type(X::x)` will return an `int`.
 *
 * @param[in]  t     An object to cast
 *
 * @tparam     T     The type of the object
 *
 * @return     The object statically casted to its underlying type.
 */
template <typename T>
constexpr auto to_underlying_type(T t) {
  return static_cast<std::underlying_type_t<T>>(t);
}

namespace details {

/**
 * @brief      Type trait which determines if a type is a std::tuple.
 */
template <typename>
struct is_tuple : public std::false_type {};

/**
 * @tparam     Types  A parameter pack of types used in a tuple. Matches any
 *                    valid `std::tuple`.
 */
template <typename... Types>
struct is_tuple<std::tuple<Types...>> : public std::true_type {};

}  // namespace details

/**
 * @brief Type trait which determines if a type is a std::tuple specialisation.
 *
 * @tparam T The type
 */
template <typename T>
struct is_tuple : public details::is_tuple<std::decay_t<T>> {};

template <typename T>
inline constexpr bool is_tuple_v = is_tuple<T>::value;

/**
 * @brief      Type trait which determines if a type is a std::vector.
 */
template <typename>
struct is_vector : public std::false_type {};

template <typename T, typename Alloc>
struct is_vector<std::vector<T, Alloc>> : public std::true_type {};

template <typename T>
inline constexpr bool is_vector_v = is_vector<T>::value;

/**
 * @brief      Type trait which determines if a type satisfies the properties of
 *             an iterable type.
 *
 * @tparam     T       The type to check
 * @tparam     Enable  A helper template parameter
 */
template <typename T, typename Enable = void>
struct is_iterable : std::false_type {};

/**
 * @brief      Valid if the template parameter `std::void_t` is well formed,
 *             when all types given to it as parameters are present/valid.
 *
 *             `std::declval` is used to check if class methods exist.
 *             `std::decay` removes const/volatile qualifiers, references,
 *             giving just the plain type.
 */
template <typename T>
struct is_iterable<
    T, std::void_t<typename std::decay_t<T>::value_type,
                   typename std::decay_t<T>::iterator,
                   decltype(std::declval<std::decay_t<T>>().begin()),
                   decltype(std::declval<std::decay_t<T>>().end())>>
    : public std::true_type {};

template <typename T>
inline constexpr bool is_iterable_v = is_iterable<T>::value;

/**
 * @brief      Type trait which determines if a type satisfies the properties of
 *             an iterable type.
 *
 * @tparam     T       The type to check
 * @tparam     Enable  A helper template parameter
 */
template <typename T, typename Enable = void>
struct is_iterable_and_clearable : std::false_type {};

/**
 * @brief      Valid if the template parameter `std::void_t` is well formed,
 *             when all types given to it as parameters are present/valid.
 *
 *             `std::declval` is used to check if class methods exist.
 *             `std::decay` removes const/volatile qualifiers, references,
 *             giving just the plain type.
 */
template <typename T>
struct is_iterable_and_clearable<
    T, std::void_t<typename std::decay_t<T>::value_type,
                   typename std::decay_t<T>::iterator,
                   decltype(std::declval<std::decay_t<T>>().begin()),
                   decltype(std::declval<std::decay_t<T>>().end()),
                   decltype(std::declval<std::decay_t<T>>().clear())>>
    : public std::true_type {};

template <typename T>
inline constexpr bool is_iterable_and_clearable_v =
    is_iterable_and_clearable<T>::value;

/**
 * @brief      Type trait which determines if a type is const-iterable.
 *
 * @tparam     T       The type to check
 * @tparam     Enable  A helper template parameter
 */
template <typename T, typename Enable = void>
struct is_const_iterable : std::false_type {};

template <typename T>
struct is_const_iterable<
    T, std::void_t<typename std::decay_t<T>::value_type,
                   typename std::decay_t<T>::const_iterator,
                   decltype(std::declval<std::decay_t<T>>().cbegin()),
                   decltype(std::declval<std::decay_t<T>>().cend())>>
    : public std::true_type {};

template <typename T>
inline constexpr bool is_const_iterable_v = is_const_iterable<T>::value;

/**
 * @brief      Type trait which determines if a type is CopyInsertable and can
 *             be appended to.
 *
 * @tparam     T       The type
 * @tparam     Enable  Template helper
 */
template <typename T, typename Enable = void>
struct has_push_back : std::false_type {};

template <typename T>
struct has_push_back<T, std::void_t<decltype(std::declval<T>().push_back(
                            typename T::value_type{}))>> : std::true_type {};

template <typename T>
inline constexpr bool has_push_back_v = has_push_back<T>::value;

/**
 * @brief      Type trait which determines if a type has some Container
 *             facilities: size, and element-wise access.
 *
 * @tparam     T          The type to check
 * @tparam     <unnamed>  Template helper
 */
template <typename T, typename = void>
struct is_location_accessible : std::false_type {};

template <typename T>
struct is_location_accessible<
    T, std::void_t<typename std::decay_t<T>::value_type,
                   typename std::decay_t<T>::size_type,
                   decltype(std::declval<T&>().size()),
                   decltype(std::declval<T&>().at(
                       std::declval<typename std::decay_t<T>::size_type>())),
                   decltype(std::declval<T&>().operator[](
                       std::declval<typename std::decay_t<T>::size_type>()))>>
    : public std::true_type {};

template <typename T>
inline constexpr bool is_location_accessible_v =
    is_location_accessible<T>::value;

/**
 * @brief      Determines if a type can be writen to an output stream (<<)
 *
 * @tparam     T          The type
 * @tparam     S          The stream
 * @tparam     <unnamed>  Template helper
 */
template <typename T, typename S = std::ostream, typename = void>
struct is_writable_to_stream : std::false_type {};

template <typename T>
struct is_writable_to_stream<
    T, std::ostream,
    std::void_t<decltype(std::declval<std::ostream&>() << std::declval<T&>())>>
    : std::true_type {};

template <typename T>
inline constexpr bool is_writable_to_stream_v = is_writable_to_stream<T>::value;

/**
 * @brief      Determines if a type can be read from an input stream (>>)
 *
 * @tparam     T          The type
 * @tparam     S          The stream
 * @tparam     <unnamed>  Template helper
 */
template <typename T, typename S = std::istream, typename = void>
struct is_readable_from_stream : std::false_type {};

template <typename T>
struct is_readable_from_stream<
    T, std::istream,
    std::void_t<decltype(std::declval<std::istream&>() >> std::declval<T&>())>>
    : std::true_type {};

template <typename T>
inline constexpr bool is_readable_from_stream_v =
    is_readable_from_stream<T>::value;

template <typename... Ts>
using unique_ptr_tuple = std::tuple<std::unique_ptr<Ts>...>;

/* The following type traits determine framework node type. Nodes were provided
 * additional inherited classes to simplify dealing with template
 * instantiations. Therefore, to check if a node is a consumer, the only check
 * necessary is to verify if the type inherits from TConsumer. */

template <typename T, typename Enable = void>
struct is_consumer : public std::false_type {};

template <typename T>
struct is_consumer<T, std::enable_if_t<std::is_base_of_v<
                          exot::framework::TConsumer, std::decay_t<T>>>>
    : public std::true_type {};

template <typename T>
inline constexpr bool is_consumer_v = is_consumer<T>::value;

template <typename T, typename Enable = void>
struct is_producer : public std::false_type {};

template <typename T>
struct is_producer<T, std::enable_if_t<std::is_base_of_v<
                          exot::framework::TProducer, std::decay_t<T>>>>
    : public std::true_type {};

template <typename T>
inline constexpr bool is_producer_v = is_producer<T>::value;

template <typename T, typename Enable = void>
struct is_processor : public std::false_type {};

template <typename T>
struct is_processor<T, std::enable_if_t<std::is_base_of_v<
                           exot::framework::TProcessor, std::decay_t<T>>>>
    : public std::true_type {};

template <typename T>
inline constexpr bool is_processor_v = is_processor<T>::value;

template <typename T, typename Enable = void>
struct has_input_interface : public std::false_type {};

template <typename T>
struct has_input_interface<T, std::enable_if_t<is_consumer_v<T>>>
    : public std::true_type {};

template <typename T>
inline constexpr bool has_input_interface_v = has_input_interface<T>::value;

template <typename T, typename Enable = void>
struct has_output_interface : public std::false_type {};

template <typename T>
struct has_output_interface<T, std::enable_if_t<is_producer_v<T>>>
    : public std::true_type {};

template <typename T>
inline constexpr bool has_output_interface_v = has_output_interface<T>::value;

/**
 * @brief      Type trait to determine if a class has a void(void) process()
 *             function
 *
 * @tparam     T          The class
 * @tparam     <unnamed>  Template helper
 */
template <typename T, typename = void>
struct has_process_function : public std::false_type {};

template <typename T>
struct has_process_function<
    T, std::void_t<decltype(std::declval<std::decay_t<T>>().process())>>
    : public std::true_type {};

template <typename T>
inline constexpr bool has_process_function_v = has_process_function<T>::value;

}  // namespace exot::utilities
