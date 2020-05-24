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
 * @file meters/cache_er.h
 * @author     Bruno Klopott
 * @brief      Evict + Reload meter, measures access time and evicts by reading
 *             memory mapped to the same set+slice.
 *
 * @note       Features in [1], used on platforms which do not have cache flush
 *             instructions.
 *
 *             [1] M. Lipp, D. Gruss, R. Spreitzer, C. Maurice, and S. Mangard,
 *             “ARMageddon - Cache Attacks on Mobile Devices.,” 23rd USENIX
 *             Security Symposium, 2016.
 *
 * @todo       This meter should be made cross-platform. At the moment (2019/04)
 *             support for cache information and address decomposing is only for
 *             x86.
 */

#pragma once

#include <cstdint>
#include <map>
#include <thread>
#include <utility>

#include <exot/meters/base_shared_memory.h>
#include <exot/primitives/cache.h>
#include <exot/utilities/configuration.h>
#include <exot/utilities/eviction.h>
#include <exot/utilities/helpers.h>
#include <exot/utilities/literals.h>
#include <exot/utilities/timing.h>

#if defined(__x86_64__)
#include <exot/primitives/tsc.h>
#else
#include <exot/utilities/timing_source.h>
#endif

namespace exot::modules {

struct cache_er : public meter_shared_memory_base {
  using map_type     = std::map<void*, void*>;
  using evicter_type = exot::utilities::Evicter;

  const char* name() const override { return "cache_er"; }

  struct settings : public exot::utilities::configurable<settings>,
                    public exot::modules::meter_shared_memory_base::settings {
    /* Since settings inherits from base's settings, we need to provide a way
     * to resolve to the right functions. The `set_json` and `configure` need to
     * work on current structure's data as well as the derived one. The same
     * pattern is used, for example, in the `meter_host`. */
    using base_t = exot::utilities::configurable<settings>;

    /* Eviction strategy settings. */
    std::size_t addresses_per_loop = 2;
    std::size_t accesses_per_loop  = 2;
    std::size_t overlap_parameter  = 1;

    const char* name() const { return "cache"; }

    void set_json(const nlohmann::json& root) {
      base_t::set_json(root);
      exot::modules::meter_shared_memory_base::settings::set_json(root);
    }

    auto describe() {
      return base_t::describe() +
             exot::modules::meter_shared_memory_base::settings::describe();
    }

    void configure() {
      base_t::bind_and_describe_data(
          "addresses_per_loop", addresses_per_loop,
          "number of addresses accessed in an eviction "
          "loop |uint|, default: 2");
      base_t::bind_and_describe_data(
          "accesses_per_loop", accesses_per_loop,
          "number of accesses to each address |uint|, default: 2");
      base_t::bind_and_describe_data(
          "overlap_parameter", overlap_parameter,
          "the eviction overlap parameter |uint|, default: 1");
      exot::modules::meter_shared_memory_base::settings::configure();
    }
  };

  explicit cache_er(settings& conf)
      : meter_shared_memory_base(conf), lconf_{conf} {
    using namespace exot::utilities;

    /* Initialise the evicter. */
    evicter_ = std::make_unique<evicter_type>(lconf_.addresses_per_loop,
                                              lconf_.accesses_per_loop,
                                              lconf_.overlap_parameter);

    const auto slice_count =
#if defined(__x86_64__)
        static_cast<unsigned>(std::thread::hardware_concurrency() / 2);
#else
        1;
#endif

    /* To get all set-slice pairs, we need at least 'line size' * 'number of
     * sets' * 'number of slices' addresses. Therefore, to get at least 2
     * of each set-slice pairs, we need twice that amount.
     *
     * For example, on a system with 64 bytes per line, 8192 sets, and 4 slices,
     * we would need 0x400000 (64 * 8192 * 4 * 2) bytes of memory. */
    const auto minimum_mem_size = llc_->coherency_line_size().value() *
                                  llc_->number_of_sets().value() *
                                  (slice_count * 2);

    /* Save the line length for use with the evicter. */
    line_length_ = llc_->coherency_line_size().value();

    /* Verify if sufficient amout of memory has been mapped. */
    if (mapping_->get_length() < minimum_mem_size) {
      debug_log_->critical(
          "[meter] to build an eviction set, a huge pages mapping of at least "
          "{:#x} bytes is required.",
          minimum_mem_size);
      throw std::logic_error("Insufficient mapping size");
    }

    /* Loop through target addresses and find matching eviction sets. A matching
     * set has the same set and slice indeces, but different address. */
    for (auto i = 0; i < address_set_info_.size(); ++i) {
      auto [target_addr, target_set, target_slice] = address_set_info_[i];

      debug_log_->debug(
          "[meter] attempting to find eviction candidate address for target "
          "address {}, set: {:#x}, slice: {:#b}",
          target_addr, target_set, target_slice);

      /* First candidate is at least 'number of sets' * 'line size' away, i.e.
       * the first address with the same set number as the target. The
       * assumption here is that the mapped memory contains contiguous addresses
       * and is sufficiently large. This method would surely not work otherwise;
       * however, this approach greatly reduces the complexity of creating an
       * eviction set. */
      auto step =
          llc_->coherency_line_size().value() * llc_->number_of_sets().value();
      auto traversed = step;  // hold the total amout of traversed memory
      auto candidate = reinterpret_cast<std::uintptr_t>(target_addr) + step;

      for (auto i = 0;; ++i) {
        auto candidate_addr          = reinterpret_cast<void*>(candidate);
        auto candidate_physical_addr = resolver_(candidate_addr);
        auto candidate_set = decomposer_->get_set(candidate_physical_addr);
        auto candidate_slice =
#if defined(__x86_64__)
            exot::primitives::slice_selection_hash(candidate_physical_addr,
                                                   slice_count);
#else
            -1;
#endif

        debug_log_->trace(
            "[meter] candidate {} -> address: {}, set: {:#x}, slice: {:#b}", i,
            candidate_addr, candidate_set, candidate_slice);

        if (candidate_slice == target_slice) {
          auto dist = [](auto lhs, auto rhs) {
            return lhs > rhs ? lhs - rhs : rhs - lhs;
          };

          debug_log_->debug(
              "[meter] adding address {} ({:#x}) to eviction set "
              "(target: {}, distance: {:#x})",
              candidate_addr, candidate_physical_addr, target_addr,
              dist(candidate_physical_addr, resolver_(target_addr)));
          eviction_map_.emplace(target_addr, candidate_addr);
          break;
        }

        candidate += step;
        traversed += step;

        /* If amount of traversed memory is out of bounds, no eviction address
         * was found, and the program will abort. */
        if (traversed > mapping_->get_length()) {
          debug_log_->critical(
              "[meter] could not find eviction address for target: {}",
              target_addr);
          throw std::logic_error("Could not find required eviction addresses");
        }
      }
    }
  }

  /**
   * @brief      Gets the eviction map.
   *
   * @return     The eviction map.
   */
  const auto& get_eviction_map() const { return eviction_map_; }

  /**
   * @brief      Gets the eviction address.
   *
   * @param      target  The target address
   *
   * @return     The eviction address.
   */
  auto get_eviction_address(void* target) { return eviction_map_.at(target); }

  /**
   * @brief      Measures the access time to an address and evicts the set in
   *             which it is located
   *
   * @param      addr  The address
   *
   * @return     The number of cycles to access the address
   */
  inline __attribute__((always_inline)) std::uint64_t perform_action_on_address(
      void* addr) override {
    /* Casting to uint8_t* such that eviction will happen at 1B increments. */
    auto* eviction_addr = reinterpret_cast<std::uint8_t*>(eviction_map_[addr]);

#if defined(__x86_64__)
    auto _ = exot::utilities::timeit<exot::primitives::MemoryFencedTSC>(
        exot::primitives::access_read<std::uint8_t>, addr);
#else
    auto _ = exot::utilities::default_timing_facility(
        exot::primitives::access_read<>, addr);
#endif
    SPDLOG_LOGGER_TRACE(debug_log_,
                        "Evicting address {} using {}B eviction set @{}", addr,
                        line_length_, fmt::ptr(eviction_addr));
    evicter_->read(eviction_addr, line_length_);
    return _;
  }

 private:
  settings lconf_;         //! local settings, different than parent's conf_
  map_type eviction_map_;  //! mapping target-addr -> eviction-addr
  std::unique_ptr<evicter_type> evicter_;  //! evicter as a unique ptr due to
                                           //! the deleted default constructor
  std::size_t line_length_;
};

}  // namespace exot::modules
