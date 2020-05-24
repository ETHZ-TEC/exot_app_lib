/**
 * @file generators/cache_ristenpart_mt.cpp
 * @author     Bruno Klopott
 * @brief      Multi-threaded generator causing LLC cache eviction.
 */

#include <exot/generators/cache_ristenpart_st.h>

#include <spdlog/spdlog.h>

#include <exot/utilities/helpers.h>

using namespace exot::modules;

generator_cache_ristenpart_st::generator_cache_ristenpart_st(
    generator_cache_ristenpart_st::settings& conf)
    : bitset_generator_base(conf), lconf_{conf} {
  using namespace exot::utilities;

  if (conf_.cores.size() != 1) {
    throw std::logic_error(
        "The module 'generator_cache_ristenpart_st' can only be used in a "
        "single-threaded scenario.");
  }

  if ((lconf_.cache_level < 2) || (lconf_.cache_level > 3)) {
    throw std::logic_error(
        "Generator can only target L2 or L3 unified caches.");
  }

  if (!is_power_of_2(lconf_.d_multiplier)) {
    throw std::logic_error("The d-factor multiplier must be a power of 2.");
  }

  /* Get information about the cache. */
  auto cache_info = (lconf_.cache_info.has_value()
                         ? CPUCacheInfo(0, lconf_.cache_info.value())
                         : CPUCacheInfo(0));
  auto cache =
      cache_info.at(lconf_.cache_level, static_cast<int>(CacheType::Unified));

  auto cache_size      = cache.size().value();
  auto cache_line_size = cache.coherency_line_size().value();

  b_        = 2ul * cache_size * 1024ul;
  d_factor_ = cache_line_size * lconf_.d_multiplier;
  buffer_.resize(b_);

  logger_->info(
      "[generator->generator_cache_ristenpart_st] using cache at level {}, "
      "line size: {}, size: {:#x}, buffer size: {:#x}, d-factor: {:#x}",
      lconf_.cache_level, cache_line_size, cache_size * 1024, b_, d_factor_);

  even_addresses_.reserve(b_ / d_factor_);
  odd_addresses_.reserve(b_ / d_factor_);

  for (auto i = 0; i < buffer_.size(); ++i) {
    if ((reinterpret_cast<std::uintptr_t>(&buffer_[i]) % (2 * d_factor_)) ==
        0) {
      even_addresses_.push_back(reinterpret_cast<std::uint8_t*>(&buffer_[i]));
    } else if ((reinterpret_cast<std::uintptr_t>(&buffer_[i]) %
                (2 * d_factor_)) == d_factor_) {
      odd_addresses_.push_back(reinterpret_cast<std::uint8_t*>(&buffer_[i]));
    }
  }

  even_addresses_.shrink_to_fit();
  odd_addresses_.shrink_to_fit();

  logger_->info(
      "[generator->generator_cache_ristenpart_st] even addresses set size: "
      "{:#x}, odd addresses set size: {:#x}",
      even_addresses_.size(), odd_addresses_.size());
}

void generator_cache_ristenpart_st::generate_load(
    const generator_cache_ristenpart_st::decomposed_type& decomposed_subtoken,
    const generator_cache_ristenpart_st::enable_flag_type& flag,
    generator_cache_ristenpart_st::core_type core,
    generator_cache_ristenpart_st::index_type index) {
  /* If tracing is enabled, we also track how many buffer traversals were
   * completed. */
#ifdef SPDLOG_TRACE_ON
  auto count = int{0};
#endif

  SPDLOG_LOGGER_TRACE(
      logger_,
      "[generator->generate_load] worker on core {} accessing {} addresses",
      core, decomposed_subtoken ? "even" : "odd");

  if (decomposed_subtoken) {
    while (flag) {
      /* Access memory pointed to by even addresses. */
      evicter_.read_ptrs(even_addresses_);
#ifdef SPDLOG_TRACE_ON
      ++count;
#endif
    }
  } else {
    while (flag) {
      /* Access memory pointed to by odd addresses. */
      evicter_.read_ptrs(odd_addresses_);
#ifdef SPDLOG_TRACE_ON
      ++count;
#endif
    }
  }

  SPDLOG_LOGGER_TRACE(logger_,
                      "[generator->generate_load] worker on core {} "
                      "completed {} loops through addresses",
                      core, count);
}
