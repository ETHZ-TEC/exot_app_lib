/**
 * @file meters/cache_fr.h
 * @author     Bruno Klopott
 * @brief      Flush + Reload meter, measures the access time and flushes the
 *             corresponding cache set.
 *
 * @note       Widely used, appears most prominently in [1].
 *
 *             [1]  Y. Yarom and K. Falkner, “FLUSH+RELOAD - A High Resolution,
 *             Low Noise, L3 Cache Side-Channel Attack.,” 23rd USENIX Security
 *             Symposium, 2014.
 */

#pragma once

#include <cstdint>

#include <exot/meters/base_shared_memory.h>
#include <exot/primitives/cache.h>

#if defined(__x86_64__)
#include <exot/primitives/tsc.h>
#else
#include <exot/utilities/timing_source.h>
#endif

#include <exot/utilities/timing.h>

namespace exot::modules {

struct cache_fr : public meter_shared_memory_base {
  using meter_shared_memory_base::meter_shared_memory_base;

  const char* name() const override { return "cache_fr"; }

  inline __attribute__((always_inline)) std::uint64_t perform_action_on_address(
      void* addr) override {
    using namespace exot::utilities;
    using namespace exot::primitives;

#if defined(__x86_64__)
#if defined(cache_fr_ASSEMBLY)
    return flush_reload(addr);
#else
    auto _ = timeit<MemoryFencedTSC>(access_read<>, addr);
    flush(addr);
    return _;
#endif
#else
    auto _ = default_timing_facility(access_read<>, addr);
    flush(addr);
    return _;
#endif
  }
};

}  // namespace exot::modules
