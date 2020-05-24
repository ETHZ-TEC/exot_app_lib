/**
 * @file generators/cache_ristenpart_st.h
 * @author     Bruno Klopott
 * @brief      Multi-threaded generator causing LLC cache eviction.
 */

#pragma once

#include <cstdint>

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

#include <exot/generators/base_bitset.h>
#include <exot/utilities/alignment.h>
#include <exot/utilities/allocator.h>
#include <exot/utilities/cache.h>
#include <exot/utilities/configuration.h>
#include <exot/utilities/eviction.h>

namespace exot::modules {

struct generator_cache_ristenpart_st
    : public exot::modules::bitset_generator_base {
  struct settings : public exot::utilities::configurable<settings>,
                    public exot::modules::bitset_generator_base::settings {
    using base_t = exot::utilities::configurable<settings>;

    unsigned d_multiplier{1};  //! d_factor / cache_line_size
    unsigned cache_level{3};   //! targeted cache level, either L2 or L3

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
      base_t::bind_and_describe_data("d_multiplier", d_multiplier,
                                     "the d factor/cache line size |uint|");
      base_t::bind_and_describe_data(
          "cache_level", cache_level,
          "the cache level to target |uint|, either 2 or 3");
      base_t::bind_and_describe_data("cache_info", cache_info,
                                     "manual cache info maps |CacheInfo[]|");
      exot::modules::bitset_generator_base::settings::configure();
    }
  };

  explicit generator_cache_ristenpart_st(settings& conf);

  void generate_load(const decomposed_type& decomposed_subtoken,
                     const enable_flag_type& flag, core_type core,
                     index_type index);

 private:
  using value_type          = std::uint8_t;
  using allocator           = exot::utilities::HugePagesAllocator<value_type>;
  using buffer_type         = std::vector<value_type, allocator>;
  using address_vector_type = std::vector<value_type*>;

  unsigned long int b_;    //! buffer size
  unsigned int d_factor_;  //! cache line size * d_multiplier

  buffer_type buffer_;
  address_vector_type even_addresses_;      //! even addresses set
  address_vector_type odd_addresses_;       //! even addresses set
  settings lconf_;                          //! local settings
  exot::utilities::Evicter evicter_{1, 1};  //! the evicter
};

}  // namespace exot::modules
