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
 * @file generators/base_shared_memory.h
 * @author     Bruno Klopott
 * @brief      Base class for generators using shared memory.
 */

#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <thread>

#include <spdlog/spdlog.h>

#include <spdlog/sinks/stdout_color_sinks.h>

#include <exot/utilities/allocator.h>
#include <exot/utilities/bits.h>
#include <exot/utilities/cache.h>
#include <exot/utilities/configuration.h>
#include <exot/utilities/filesystem.h>
#include <exot/utilities/helpers.h>
#include <exot/utilities/mmap.h>
#include <exot/utilities/pagemap.h>

#if defined(__x86_64__)
#include <thread>

#include <exot/primitives/cache.h>
#include <exot/primitives/x86_64.h>
#endif

#ifndef shared_memory_generator_base_CLEVER_FOR_LOOP
#define shared_memory_generator_base_CLEVER_FOR_LOOP true
#endif

#ifndef shared_memory_generator_base_REPEAT
#define shared_memory_generator_base_REPEAT 1
#endif

#ifndef shared_memory_generator_base_YIELD_IN_GENERATE
#define shared_memory_generator_base_YIELD_IN_GENERATE true
#endif

#if defined(SPDLOG_TRACE_ON)
static auto count{0ull};
#define INCREMENT_COUNT() ++count
#else
#define INCREMENT_COUNT()
#endif

namespace exot::modules {

struct shared_memory_generator_base {
  using subtoken_type    = uint_fast64_t;  //
  using decomposed_type  = uint_fast64_t;  //
  using core_type        = unsigned;       // core number
  using index_type       = std::size_t;    // thread index
  using enable_flag_type = std::atomic_bool;
  using logger_pointer   = std::shared_ptr<spdlog::logger>;

  /**
   * @brief      Performs an action on an address from the address set
   * @note       Must be implemented by deriving classes.
   *
   * @param      address  The address
   */
  virtual void perform_action_on_address(void* address) = 0;

  struct options {
    static const bool clever_for_loop =
        shared_memory_generator_base_CLEVER_FOR_LOOP;
    static const std::size_t repetitions = shared_memory_generator_base_REPEAT;
    static const bool yield_in_generate =
        shared_memory_generator_base_YIELD_IN_GENERATE;
  };

  struct settings : public exot::utilities::configurable<settings> {
    std::string shm_file;            //! shared memory file
    bool use_huge_pages    = false;  //! use huge pages?
    unsigned set_count     = 1u;   //! number of LLC sets used for communication
    unsigned set_increment = 64u;  //! number of cache-line-sized spaces
                                   //! between consecutive  communication sets,
                                   //! default: page offsets (64 * line = 4096)

    using cache_maps_t =
        std::vector<typename exot::utilities::CacheInfo::map_type>;
    std::optional<cache_maps_t> cache_info;

    const char* name() const { return "generator"; }
    void configure() {
      bind_and_describe_data(
          "shm_file", shm_file,
          "the shared memory file |str|, e.g. /mnt/shm/file");
      bind_and_describe_data("use_huge_pages", use_huge_pages,
                             "map memory with the huge pages flag? |bool|");
      bind_and_describe_data(
          "set_count", set_count,
          "the number of sets to use |uint|, in range [1, 64]");
      bind_and_describe_data("set_increment", set_increment,
                             "choose every nth set |uint|, e.g. 16");
      bind_and_describe_data("cache_info", cache_info,
                             "manual cache info maps |CacheInfo[]|");
    }
  };

  explicit shared_memory_generator_base(settings& conf) : conf_{conf} {
    using namespace exot::utilities;

    if (conf_.shm_file.empty())
      throw std::logic_error("Shared memory file must be provided!");
    if (!exot::utilities::exists(conf_.shm_file))
      throw std::logic_error("Shared memory file must exist!");
    if (!exot::utilities::is_readable(conf_.shm_file))
      throw std::logic_error("Shared memory file must be readable!");
    if (conf_.set_count == 0)
      throw std::logic_error("At least 1 set is needed for the channel!");
    if (conf_.set_count > 64)
      throw std::logic_error(
          "Shared memory-based generators support only up to 64 lines!");
    if (conf_.set_increment < 1)
      throw std::logic_error("Set increment must be at least 1!");

    validator_ = conf_.set_count == 64
                     ? std::numeric_limits<std::uint_fast64_t>::max()
                     : (std::uint_fast64_t{1} << conf_.set_count) -
                           std::uint_fast64_t{1};

    try {
      auto prot =
          std::initializer_list<MMapProt>{MMapProt::Read, MMapProt::Write};
      auto flag = conf_.use_huge_pages
                      ? std::initializer_list<MMapFlag>{MMapFlag::Shared,
                                                        MMapFlag::HugeTable}
                      : std::initializer_list<MMapFlag>{MMapFlag::Shared};
      /* May throw if unsuccessful. */
      logger_->debug("[generator] creating mapping of {}", conf_.shm_file);
      mapping_ = std::make_unique<MMap>(conf_.shm_file, prot, flag);
    } catch (const std::exception& e) {
      throw std::logic_error(fmt::format(
          "Mapping shared memory failed.{}\n   why():  {}",
          conf_.use_huge_pages ? " Make sure to provide a huge pages "
                                 "shared memory, e.g. via hugetlbfs."
                               : "",
          e.what()));
    }

    /* Advice the kernel that the memory will be accessed randomly, hoping that
     * would prevent unintended prefetching of consecutive addresses. */
    logger_->debug("[generator] advising the kernel about the mapped mem.");
    advise(mapping_->get_address(), mapping_->get_length(), Advice::random);

    auto llc = (conf_.cache_info.has_value()
                    ? CPUCacheInfo(0, conf_.cache_info.value())
                    : CPUCacheInfo(0))
                   .llc();
    auto resolver   = AddressTranslator();
    auto decomposer = AddressDecomposer(llc);

    base_       = reinterpret_cast<std::uintptr_t>(mapping_->get_address());
    auto offset = static_cast<std::uintptr_t>(0ull);

    logger_->debug(
        "[generator] created mapping of size: {:#x} from address: {:#x} "
        "(physical: {:#x})",
        mapping_->get_length(), base_,
        resolver(reinterpret_cast<void*>(base_)));

#if defined(__x86_64__)
    /* Only hyper-threaded multicore platforms are supported, hence '/ 2'. */
    const auto core_count = std::thread::hardware_concurrency() / 2;
#endif

    for (auto i = 0u; i < conf_.set_count; ++i) {
      auto addr      = base_ + offset;
      auto phys      = resolver(reinterpret_cast<void*>(addr));
      auto set_index = decomposer.get_set(phys);
#if defined(__x86_64__)
      auto slice_index =
          exot::primitives::slice_selection_hash(phys, core_count);
#else
      auto slice_index = -1;
#endif

      logger_->debug(
          "[generator] adding address {:#x} ({:#x}) to address set, offset "
          "from base: {:#8x}, set/index: {:#6x}"
#if defined(__x86_64__)
          ", slice: {:#b}"
#endif
          ,
          addr, phys, offset, set_index
#if defined(__x86_64__)
          ,
          slice_index
#endif
      );

      address_set_.push_back(reinterpret_cast<void*>(addr));
      offset += llc.coherency_line_size().value() * conf_.set_increment;
    }
  }

  inline decomposed_type decompose_subtoken(const subtoken_type& subtoken,
                                            core_type core, index_type index) {
    return subtoken;
  }

  inline bool validate_subtoken(const subtoken_type& subtoken) {
    return subtoken <= validator_;
  }

  /**
   * @note       Must be implemented by inheriting classes.
   */
  inline void generate_load(const decomposed_type& decomposed_subtoken,
                            const enable_flag_type& flag, core_type core,
                            index_type index) {
    /* Skip if no bit set in decompose_subtoken. */
    if (decomposed_subtoken != 0) {
      SPDLOG_LOGGER_TRACE(logger_,
                          "Decomposed non-zero token on core {}: {:#b}", core,
                          decomposed_subtoken);

#if defined(SPDLOG_TRACE_ON)
      count = 0ull;
#endif

      /* Access until flag is set to false. */
      while (flag) {
        generate_load_helper(decomposed_subtoken);
        if constexpr (options::yield_in_generate) { std::this_thread::yield(); }
      }  // while(flag)

      SPDLOG_LOGGER_TRACE(logger_, "Performed {} actions on core {}",
                          count * options::repetitions, core);
    }  // if (decomposed_subtoken != 0)
  }

  /**
   * @brief      Generates "load" on the address set according to the subtoken
   *
   * @param[in]  decomposed_subtoken  The decomposed subtoken
   */
  inline __attribute__((always_inline)) void generate_load_helper(
      const decomposed_type& decomposed_subtoken) {
    /* Method 1: efficiently access set bits in decomposed_subtoken. */
    if constexpr (options::clever_for_loop) {
      exot::utilities::loop_through_set_bits(
          decomposed_subtoken, [&](std::size_t i) {
            INCREMENT_COUNT();
            exot::utilities::const_for<0, options::repetitions>(
                [&, this](const auto) {
                  perform_action_on_address(address_set_[i]);
                });
          });
    }
    /* Method 2: iterate and test individual bits. */
    else {
      for (auto i{0u}; i < address_set_.size(); ++i) {
        if (exot::utilities::test_bit(decomposed_subtoken, i)) {
          INCREMENT_COUNT();
          exot::utilities::const_for<0, options::repetitions>(
              [&, this](const auto) {
                perform_action_on_address(address_set_[i]);
              });
        }
      }
    }  // if constexpr (options::clever_for_loop)
  }

  /**
   * @brief      Gets the address set.
   *
   * @return     The address set.
   */
  const auto& get_address_set() const { return address_set_; }

 protected:
  settings conf_;
  logger_pointer logger_ =
      spdlog::get("log") ? spdlog::get("log") : spdlog::stderr_color_mt("log");

  uint_fast64_t validator_;

  std::unique_ptr<exot::utilities::MMap> mapping_;
  std::vector<void*> address_set_;
  std::uintptr_t base_;
};

}  // namespace exot::modules
