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
#pragma once

#include <memory>

#include <exot/generators/base_shared_memory.h>
#include <exot/primitives/cache.h>
#include <exot/utilities/configuration.h>
#include <exot/utilities/eviction.h>

namespace exot::modules {

/**
 * @brief      Generator that accesses addresses by performing reads
 */
struct generator_cache_read_st : public shared_memory_generator_base {
  using shared_memory_generator_base::shared_memory_generator_base;

  inline __attribute__((always_inline)) void perform_action_on_address(
      void* addr) override {
    exot::primitives::access_read<>(addr);
  }
};

/**
 * @brief      Generator that accesses addresses by performing writes
 */
struct generator_cache_write_st : public shared_memory_generator_base {
  using shared_memory_generator_base::shared_memory_generator_base;

  inline __attribute__((always_inline)) void perform_action_on_address(
      void* addr) override {
    exot::primitives::access_write<>(addr);
  }
};

/**
 * @brief      Generator that accesses addresses with eviction patterns
 */
struct generator_cache_evict_st : public shared_memory_generator_base {
  using evicter_type = exot::utilities::Evicter;

  struct settings
      : public exot::utilities::configurable<settings>,
        public exot::modules::shared_memory_generator_base::settings {
    using base_t = exot::utilities::configurable<settings>;

    /* Eviction strategy settings. */
    std::size_t addresses_per_loop{2};
    std::size_t accesses_per_loop{2};
    std::size_t overlap_parameter{1};
    std::size_t eviction_length{64};

    const char* name() const { return "generator"; }

    void set_json(const nlohmann::json& root) {
      base_t::set_json(root);
      exot::modules::shared_memory_generator_base::settings::set_json(root);
    }

    auto describe() {
      return base_t::describe() +
             exot::modules::shared_memory_generator_base::settings::describe();
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
      base_t::bind_and_describe_data(
          "eviction_length", eviction_length,
          "number of bytes to evict |uint|, default: 64");
      exot::modules::shared_memory_generator_base::settings::configure();
    }
  };

  explicit generator_cache_evict_st(settings& conf)
      : shared_memory_generator_base(conf), lconf_{conf} {
    evicter_ = std::make_unique<evicter_type>(lconf_.addresses_per_loop,
                                              lconf_.accesses_per_loop,
                                              lconf_.overlap_parameter);
  }

  inline __attribute__((always_inline)) void perform_action_on_address(
      void* addr) override {
    evicter_->read(reinterpret_cast<std::uint8_t*>(addr),
                   lconf_.eviction_length);
  }

 private:
  settings lconf_;                         //! local settings (not base's)
  std::unique_ptr<evicter_type> evicter_;  //! evicter as a unique ptr due to
                                           //! the deleted default constructor
};

}  // namespace exot::modules
