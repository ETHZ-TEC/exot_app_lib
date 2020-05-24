/**
 * @file meters/cache_fp.h
 * @author     Bruno Klopott
 * @brief      Flush + Prefetch meter, measures the access time and flushes the
 *             corresponding cache set.
 *
 * @note       Apprears in sec. 7, p. 376 in  D. Gruss, C. Maurice, A. Fogh,
 *             M. Lipp, and S. Mangard, “Prefetch Side-Channel Attacks:
 *             Bypassing SMAP and kernel ASLR,” presented at the Proceedings of
 *             the ACM Conference on Computer and Communications Security, 2016,
 *             vol. 24, pp. 368–379.
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

struct cache_fp : public meter_shared_memory_base {
  using meter_shared_memory_base::meter_shared_memory_base;

  const char* name() const override { return "cache_fp"; }

  inline __attribute__((always_inline)) std::uint64_t perform_action_on_address(
      void* addr) override {
    using namespace exot::utilities;
    using namespace exot::primitives;

#if defined(__x86_64__)
    auto _ = timeit<MemoryFencedPrefetchTSC>(prefetch, addr);
    flush(addr);
    return _;
#else
    auto _ = default_timing_facility(prefetch, addr);
    flush(addr);
    return _;
#endif
  }
};

}  // namespace exot::modules
