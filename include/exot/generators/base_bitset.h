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
 * @file generators/base_bitset.h
 * @author     Bruno Klopott
 * @brief      Generator base class using the bitset parsing method.
 */

#pragma once

#include <atomic>
#include <bitset>
#include <set>

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <spdlog/spdlog.h>

#include <spdlog/sinks/stdout_color_sinks.h>

#include <exot/utilities/configuration.h>
#include <exot/utilities/ostream.h>

namespace exot::modules {

/**
 * @brief      The bitset generator base class.
 * @note       Many generators share the same input interface: the activation is
 *             "binary", and determined by a bit value set on an unsigned
 * number.
 */
struct bitset_generator_base {
  using subtoken_type    = unsigned;     // converted to bitset
  using core_type        = unsigned;     // core number
  using decomposed_type  = bool;         // determines if thread will be busy
  using index_type       = std::size_t;  // thread index
  using enable_flag_type = std::atomic_bool;
  using logger_pointer   = std::shared_ptr<spdlog::logger>;

  struct settings : public exot::utilities::configurable<settings> {
    std::set<unsigned> cores;
    const char* name() const { return "generator"; }
    void configure() {
      bind_and_describe_data(
          "cores", cores, "cores to run workers on |uint[]|, e.g. [1, 2, 3]");
    }
  };

  explicit bitset_generator_base(settings& conf) : conf_{conf} {
    SPDLOG_LOGGER_DEBUG(logger_, "[generator] cores: {}", conf_.cores);
  }

  inline decomposed_type decompose_subtoken(const subtoken_type& subtoken,
                                            core_type core, index_type index) {
    bitset_ = subtoken;
    SPDLOG_LOGGER_TRACE(logger_,
                        "[generator->decompose_subtoken] c: {}, i: {} -> "
                        "decomposed subtoken: {}, enable: {}",
                        core, index, subtoken, bitset_[index]);
    return static_cast<bool>(bitset_[index]);
  }

  inline bool validate_subtoken(const subtoken_type& subtoken) {
    static unsigned int validator = conf_.cores.size() > 0
                                        ? (1 << conf_.cores.size()) - 1
                                        : std::numeric_limits<unsigned>::max();
    return subtoken <= validator;
  }

  /**
   * @note       Must be implemented by inheriting classes.
   */
  virtual inline void generate_load(const decomposed_type& decomposed_subtoken,
                                    const enable_flag_type& flag,
                                    core_type core, index_type index) = 0;

 protected:
  std::bitset<16> bitset_;
  settings conf_;
  logger_pointer logger_ =
      spdlog::get("log") ? spdlog::get("log") : spdlog::stderr_color_mt("log");
};

}  // namespace exot::modules
