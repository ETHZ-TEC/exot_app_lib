/**
 * @file meters/base.h
 * @author     Bruno Klopott
 * @brief      Definition of the module base class and template helpers.
 */

#pragma once

#include <string>       // for std::string
#include <type_traits>  // for enable_if_t, void_t, declval, true_type, false_type

namespace exot::modules {

/**
 * @brief      Default measurement units
 */
namespace DefaultUnits {
static inline constexpr const char* temperature = "°C";
static inline constexpr const char* frequency   = "kHz";
static inline constexpr const char* power       = "W";
static inline constexpr const char* process     = "";
}  // namespace DefaultUnits

/**
 * @brief      Base class from which meter modules inherit
 * @details    At the moment the class is only used for potential type trait
 *             operations.
 */
struct module {};

/**
 * @brief      Type trait for determining if a type inherits from module
 *
 * @tparam     T          The type
 * @tparam     <unnamed>  Template helper
 */
template <typename T, typename = void>
struct is_meter_module : std::false_type {};

template <typename T>
struct is_meter_module<
    T,
    std::enable_if_t<std::is_base_of_v<exot::modules::module, std::decay_t<T>>>>
    : std::true_type {};

template <typename T>
inline constexpr bool is_meter_module_v = is_meter_module<T>::value;

/**
 * @brief      Type trait for determining if a type contains a measure function
 *
 * @tparam     T          The type
 * @tparam     <unnamed>  Template helper
 */
template <typename T, typename = std::void_t<>>
struct has_meter_function : std::false_type {};

template <typename T>
struct has_meter_function<T,
                          std::void_t<decltype(std::declval<T&>().measure())>>
    : std::true_type {};

template <typename T>
inline constexpr bool has_meter_function_v = has_meter_function<T>::value;

}  // namespace exot::modules
