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
