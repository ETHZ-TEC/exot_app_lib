/**
 * @file meters/cache_ristenpart.h
 * @author     Bruno Klopott
 * @brief      Measures time to access disjoint cache sets.
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <spdlog/sinks/stdout_color_sinks.h>

#include <exot/meters/base.h>
#include <exot/utilities/allocator.h>
#include <exot/utilities/cache.h>
#include <exot/utilities/configuration.h>
#include <exot/utilities/eviction.h>

namespace exot::modules {

/**
 * @brief      The empirical frequency meter
 */
struct cache_ristenpart : module {
  using return_type = long int;

  /**
   * @brief      Settings supported by the module
   */
  struct settings : public exot::utilities::configurable<settings> {
    unsigned d_multiplier{1};  //! d_factor / cache_line_size
    unsigned cache_level{3};   //! targeted cache level, either L2 or L3

    using cache_maps_t =
        std::vector<typename exot::utilities::CacheInfo::map_type>;
    std::optional<cache_maps_t> cache_info;

    const char* name() const { return "cache_ristenpart"; }

    /* @brief The JSON configuration function */
    void configure() {
      bind_and_describe_data("d_multiplier", d_multiplier,
                             "the d factor/cache line size |uint|");
      bind_and_describe_data("cache_level", cache_level,
                             "the cache level to target |uint|, either 2 or 3");
      bind_and_describe_data("cache_info", cache_info,
                             "manual cache info maps |CacheInfo[]|");
    }
  };

  /**
   * @brief      Constructs the meter module
   *
   * @param      conf  The module settings structure
   */
  explicit cache_ristenpart(settings&);

  ~cache_ristenpart() = default;

  /**
   * @brief      Perform measurement
   *
   * @return     A vector with time measurements (in cycles)
   */
  return_type measure();

  /**
   * @brief      Get descriptions of return values
   * @details    The descriptions contain variable names and units
   *
   * @return     A vector with descriptions
   */
  std::vector<std::string> header();

 private:
  using value_type          = std::uint8_t;
  using allocator           = exot::utilities::HugePagesAllocator<value_type>;
  using buffer_type         = std::vector<value_type, allocator>;
  using address_vector_type = std::vector<value_type*>;

  settings conf_;

  unsigned long int b_;    //! buffer size
  unsigned int d_factor_;  //! cache line size * d_multiplier

  buffer_type buffer_;
  address_vector_type even_addresses_;  //! even addresses set
  address_vector_type odd_addresses_;   //! even addresses set

  exot::utilities::Evicter evicter_{1, 1};  //! the evicter

  using logger_pointer = std::shared_ptr<spdlog::logger>;
  logger_pointer debug_log_ =
      spdlog::get("log") ? spdlog::get("log") : spdlog::stderr_color_mt("log");
};

}  // namespace exot::modules
