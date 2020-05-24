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
