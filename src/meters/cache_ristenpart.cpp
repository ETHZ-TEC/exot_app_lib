/**
 * @file meters/cache_ristenpart.cpp
 * @author     Bruno Klopott
 * @brief      Implementation of the L1 cache meter.
 */

#include <exot/meters/cache_ristenpart.h>

#include <exot/utilities/helpers.h>
#include <exot/utilities/timing.h>

#if defined(__x86_64__)
#include <exot/primitives/tsc.h>
#else
#include <exot/utilities/timing_source.h>
#endif

using namespace exot::modules;

/* The order of variables in the initialiser list matters, and is defined from
 * last to first. */
cache_ristenpart::cache_ristenpart(cache_ristenpart::settings& conf)
    : conf_{conf} {
  using namespace exot::utilities;

  if ((conf_.cache_level < 2) || (conf_.cache_level > 3)) {
    throw std::logic_error(
        "Generator can only target L2 or L3 unified caches.");
  }

  if (!is_power_of_2(conf_.d_multiplier)) {
    throw std::logic_error("The d-factor multiplier must be a power of 2.");
  }

  /* Get information about the cache. */
  auto cache_info =
      (conf_.cache_info.has_value() ? CPUCacheInfo(0, conf_.cache_info.value())
                                    : CPUCacheInfo(0));
  auto cache =
      cache_info.at(conf_.cache_level, static_cast<int>(CacheType::Unified));

  auto cache_size      = cache.size().value();
  auto cache_line_size = cache.coherency_line_size().value();

  b_        = cache_size * 1024ul / 2ul;
  d_factor_ = cache_line_size * conf_.d_multiplier;
  buffer_.resize(b_);

  debug_log_->info(
      "[cache_ristenpart] using cache at level {}, line size: {}, size: {:#x}, "
      "buffer size: {:#x}, d-factor: {:#x}",
      conf_.cache_level, cache_line_size, cache_size * 1024, b_, d_factor_);

  even_addresses_.reserve(b_ / d_factor_);
  odd_addresses_.reserve(b_ / d_factor_);

  for (auto i = 0; i < buffer_.size(); ++i) {
    if ((reinterpret_cast<std::uintptr_t>(&buffer_[i]) % (2 * d_factor_)) ==
        0) {
      even_addresses_.push_back(reinterpret_cast<value_type*>(&buffer_[i]));
    } else if ((reinterpret_cast<std::uintptr_t>(&buffer_[i]) %
                (2 * d_factor_)) == d_factor_) {
      odd_addresses_.push_back(reinterpret_cast<value_type*>(&buffer_[i]));
    }
  }

  even_addresses_.shrink_to_fit();
  odd_addresses_.shrink_to_fit();

  debug_log_->info(
      "[cache_ristenpart] even addresses set size: {:#x}, odd addresses set "
      "size: {:#x}",
      even_addresses_.size(), odd_addresses_.size());
}

typename cache_ristenpart::return_type cache_ristenpart::measure() {
#if defined(__x86_64__)
  volatile auto even_time =
      exot::utilities::timeit<exot::primitives::MemoryFencedTSC>(
          [this] { evicter_.read_ptrs(even_addresses_); });

  volatile auto odd_time =
      exot::utilities::timeit<exot::primitives::MemoryFencedTSC>(
          [this] { evicter_.read_ptrs(odd_addresses_); });
#else
  volatile auto even_time = exot::utilities::default_timing_facility(
      [this] { evicter_.read_ptrs(even_addresses_); });

  volatile auto odd_time = exot::utilities::default_timing_facility(
      [this] { evicter_.read_ptrs(odd_addresses_); });
#endif

  return static_cast<return_type>(even_time) -
         static_cast<return_type>(odd_time);
}

std::vector<std::string> cache_ristenpart::header() {
  return {exot::utilities::generate_header(conf_.name(), "access_time_diff", "",
                                           "cycles")};
}
