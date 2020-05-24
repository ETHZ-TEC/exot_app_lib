/**
 * @file meters/cache_ff.h
 * @author     Bruno Klopott
 * @brief      Flush + Flush meter, measures the duration of the cache flushing
 *             operation.
 *
 * @note       D. Gruss, C. Maurice, K. Wagner, and S. Mangard,
 *             “Flush+Flush: A Fast and Stealthy Cache Attack,” in Research in
 *             Attacks, Intrusions, and Defenses, vol. 9721, no. 3, Cham:
 *             Springer International Publishing, 2016, pp. 279–299.
 */

#pragma once

#include <cstdint>

#include <exot/meters/base_shared_memory.h>
#include <exot/primitives/cache.h>
#include <exot/utilities/timing.h>

#if defined(__x86_64__)
#include <exot/primitives/tsc.h>
#else
#include <exot/utilities/timing_source.h>
#endif

namespace exot::modules {

struct cache_ff : public meter_shared_memory_base {
  using meter_shared_memory_base::meter_shared_memory_base;

  const char* name() const override { return "cache_ff"; }

  inline __attribute__((always_inline)) std::uint64_t perform_action_on_address(
      void* addr) override {
    using namespace exot::utilities;
    using namespace exot::primitives;

#if defined(__x86_64__)
    return timeit<MemoryFencedSerialisingFlushTSC>(flush, addr);
#else
    return default_timing_facility(flush, addr);
#endif
  }
};

}  // namespace exot::modules
