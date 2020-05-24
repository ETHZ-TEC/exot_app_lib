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
 * @file generators/cache_maurice_mt.h
 * @author     Bruno Klopott
 * @brief      Multi-threaded generator causing LLC cache eviction.
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <mutex>
#include <vector>

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

#include <exot/generators/base_bitset.h>
#include <exot/utilities/alignment.h>
#include <exot/utilities/allocator.h>
#include <exot/utilities/cache.h>
#include <exot/utilities/configuration.h>
#include <exot/utilities/literals.h>

namespace exot::modules {

struct generator_cache_maurice_mt
    : public exot::modules::bitset_generator_base {
  struct settings : public exot::utilities::configurable<settings>,
                    public exot::modules::bitset_generator_base::settings {
    /* Since settings inherits from base's settings, we need to provide a way
     * to resolve to the right functions. The `set_json` and `configure` need to
     * work on current structure's data as well as the derived one. The same
     * pattern is used, for example, in the `meter_host`. */
    using base_t = exot::utilities::configurable<settings>;

    std::set<unsigned> cores;
    double overprovision_factor{1.0};

    using cache_maps_t =
        std::vector<typename exot::utilities::CacheInfo::map_type>;
    std::optional<cache_maps_t> cache_info;

    const char* name() const { return "generator"; }

    void set_json(const nlohmann::json& root) {
      base_t::set_json(root);
      exot::modules::bitset_generator_base::settings::set_json(root);
    }

    auto describe() {
      return base_t::describe() +
             exot::modules::bitset_generator_base::settings::describe();
    }

    void configure() {
      base_t::bind_and_describe_data(
          "cores", cores, "cores to run workers on |uint[]|, e.g. [0, 2]");
      base_t::bind_and_describe_data(
          "overprovision_factor", overprovision_factor,
          "the memory overprovisioning factor |float|, e.g. 1.2");
      base_t::bind_and_describe_data("cache_info", cache_info,
                                     "manual cache info maps |CacheInfo[]|");
      exot::modules::bitset_generator_base::settings::configure();
    }
  };

  explicit generator_cache_maurice_mt(settings& conf);

  void generate_load(const decomposed_type& decomposed_subtoken,
                     const enable_flag_type& flag, core_type core,
                     index_type index);

 private:
  /**
   * Parameters as defined by [1] in Algorithm 1 (p. 8):
   *
   * - n : number of LLC ways;
   * - o : log2(LLC line size), width of the 'offset' field in the address;
   * - s : log2(LLC set count), width of the 'set' field in the address;
   * - c : "multiplicative parameter c", overprovisioning of the buffer;
   * - b : size of the buffer to allocate.
   *
   * Ignored parameters:
   *
   * - w : delay before sending a '1', or time to send '0'. This generator
   *   uses strict timing and executes the memory access loop continuously
   *   until a flag has been cleared by the generator_host.
   *
   * [1] C. Maurice, C. Neumann, O. Heen, and A. Francillon, “C5: Cross-Cores
   * Cache Exot Channel.,” DIMVA, vol. 9148, no. 3, pp. 46–64, 2015.
   */
  double c_;  //! multiplicative parameter
  int n_;     //! number of LLC ways
  int o_;     //! offset field width
  int s_;     //! set/index field width
  int m_;     //! stores n_ * c_
  int b_;     //! buffer size

  /* Allocate a buffer of single byte units. */
  using T = std::uint8_t;
  /* Allocate the buffer at page boundaries. */
  using Alloc =
      exot::utilities::AlignedAllocator<std::uint8_t,
                                        exot::utilities::operator""_KiB(4)>;

  inline thread_local static std::once_flag once_flag_;      //! per thread flag
  inline thread_local static std::vector<T, Alloc> buffer_;  //! per thread buf

  settings lconf_;  //! local settings
};

}  // namespace exot::modules
