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
