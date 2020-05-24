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
 * @file generators/base.h
 * @author     Bruno Klopott
 * @brief      Definition of the loadgen base class and template helpers.
 */

#pragma once

#include <type_traits>

#include <exot/utilities/configuration.h>

namespace exot::modules {

/**
 * @brief      Determine if a type is a valid generator type.
 * @details    A class is deemed a valid generator module, if it has a few
 *             member types (subtoken_type, core_type, index_type,
 *             decomposed_type, enable_flag_type, and settings), and at least 3
 *             main methods (decompose_subtoken, validate_subtoken,
 *             generate_load), with appropriate signatures.
 *
 * @tparam     T          The type to check
 * @tparam     <unnamed>  Template helper
 */
template <typename T, typename = void>
struct is_generator_module : std::false_type {};

template <typename T>
struct is_generator_module<
    T,
    std::void_t<
        typename std::decay_t<T>::subtoken_type,
        typename std::decay_t<T>::core_type,
        typename std::decay_t<T>::index_type,
        typename std::decay_t<T>::decomposed_type,
        typename std::decay_t<T>::enable_flag_type,
        typename std::decay_t<T>::settings,
        decltype(std::declval<T&>().decompose_subtoken(
            std::declval<typename std::decay_t<T>::subtoken_type const&>(),
            std::declval<typename std::decay_t<T>::core_type>(),
            std::declval<typename std::decay_t<T>::index_type>())),
        decltype(std::declval<T&>().validate_subtoken(
            std::declval<typename std::decay_t<T>::subtoken_type const&>())),
        decltype(std::declval<T&>().generate_load(
            std::declval<typename std::decay_t<T>::decomposed_type const&>(),
            std::declval<typename std::decay_t<T>::enable_flag_type const&>(),
            std::declval<typename std::decay_t<T>::core_type>(),
            std::declval<typename std::decay_t<T>::index_type>()))>>
    : public std::true_type {};

/**
 * @brief      The generator base class.
 * @note       It is not at all necessary to derive from the generator_base
 *             class, it mostly serves as an example of a valid class with
 *             appropriate function signatures, and provides documentation.
 */
struct generator_base {
  using subtoken_type    = unsigned;          //! REQUIRED, subtoken type
  using decomposed_type  = bool;              //! REQUIRED, determines if active
  using core_type        = unsigned;          //! REQUIRED, must be 'unsigned'
  using index_type       = std::size_t;       //! REQUIRED, must be 'size_t'
  using enable_flag_type = std::atomic_bool;  //! REQUIRED, must be atomic_bool

  /**
   * @brief      Generator settings
   * @note       By using the same configuration namespace as the
   *             generator_host, we can also access all of its values.
   *             Generators are allowed to hold their own configuration.
   */
  struct settings : public exot::utilities::configurable<settings> {
    const char* name() const { return "generator"; }
    void configure() {}
  };

  /**
   * @brief      Generator constructor.
   * @note       Similarly to metering modules and components, the settings
   *             object is passed to the constructor in the generator_host,
   *             therefore a constructor with such a signature must be
   *             available.
   *
   * @param      conf  The conf
   */
  explicit generator_base(settings& conf) {}

  /**
   * @brief      Decomposes the subtoken and determines activation
   *
   * @param[in]  subtoken  The subtoken
   * @param[in]  core      The CPU core
   * @param[in]  index     The thread index
   *
   * @return     The decomposed token
   */
  inline decomposed_type decompose_subtoken(const subtoken_type& subtoken,
                                            core_type core, index_type index);

  /**
   * @brief      Validates the subtoken.
   *
   * @param[in]  subtoken  The subtoken
   *
   * @return     True if valid, false otherwise.
   */
  inline bool validate_subtoken(const subtoken_type& subtoken);

  /**
   * @brief      Generates the medium-specific load.
   *
   * @param[in]  decomposed_subtoken  The decomposed subtoken
   * @param[in]  flag                 The activation atomic flag
   * @param[in]  core                 The CPU core
   * @param[in]  index                The thread index
   */
  inline void generate_load(const decomposed_type& decomposed_subtoken,
                            const enable_flag_type& flag, core_type core,
                            index_type index);
};

}  // namespace exot::modules
