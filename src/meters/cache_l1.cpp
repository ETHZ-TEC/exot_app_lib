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
 * @file meters/cache_l1.cpp
 * @author     Bruno Klopott
 * @brief      Implementation of the L1 cache meter.
 */

#include <exot/meters/cache_l1.h>

#include <cmath>
#include <type_traits>

#include <exot/utilities/allocator.h>
#include <exot/utilities/configuration.h>
#include <exot/utilities/literals.h>
#include <exot/utilities/thread.h>
#include <exot/utilities/timing.h>

#if defined(__x86_64__)
#include <exot/primitives/tsc.h>
#else
#include <exot/utilities/timing_source.h>
#endif

using Thread = exot::utilities::ThreadTraits;

using namespace exot::modules;
using namespace exot::utilities::literals;
using namespace std::chrono_literals;

namespace details {

/**
 * Helper to bootstrap cores in the constructor initialiser list, needed for
 * initialising the barrier, which copy/assignment constructors are deleted.
 */
cache_l1::settings bootstrap(cache_l1::settings& conf) {
  /* If the cores vector is empty, choose every other core on x86 architectures
   * (which likely have hyperthreading), and every core on ARM-based platforms.
   */
  if (conf.cores.empty()) {
    for (unsigned i{0}; i < std::thread::hardware_concurrency();) {
      conf.cores.push_back(i);

#if defined(__ANDROID__) || defined(__arm__) || defined(__aarch64__)
      ++i;
#else
      ++++i;
#endif
    }
  } else {
    exot::utilities::bootstrap_cores(conf.cores);
  }

  return conf;
}

}  // namespace details

/* Allocate at 32_KiB boundaries to prevent false sharing. */
using allocator   = exot::utilities::AlignedAllocator<std::uint8_t, 32_KiB>;
using buffer_type = std::vector<std::uint8_t, allocator>;

/* Timed access functions with different optimisation levels. */

inline __attribute__((always_inline, optimize("-O0"))) std::uint64_t
timed_access_O0(buffer_type& buffer, typename buffer_type::value_type& read,
                int n, int os) {
#if defined(__x86_64__)
  return exot::utilities::timeit<exot::primitives::MemoryFencedTSC>(
      [&]() mutable {
        for (auto i = 0; i < n; ++i) { read += buffer[(1 << os) * i]; }
        exot::utilities::do_not_optimise(read);
      });
#else
  return exot::utilities::default_timing_facility([&]() mutable {
    for (auto i = 0; i < n; ++i) { read += buffer[(1 << os) * i]; }
    exot::utilities::do_not_optimise(read);
  });
#endif
}

inline __attribute__((always_inline, optimize("-O1"))) std::uint64_t
timed_access_O1(buffer_type& buffer, typename buffer_type::value_type& read,
                int n, int os) {
#if defined(__x86_64__)
  return exot::utilities::timeit<exot::primitives::MemoryFencedTSC>(
      [&]() mutable {
        for (auto i = 0; i < n; ++i) { read += buffer[(1 << os) * i]; }
        exot::utilities::do_not_optimise(read);
      });
#else
  return exot::utilities::default_timing_facility([&]() mutable {
    for (auto i = 0; i < n; ++i) { read += buffer[(1 << os) * i]; }
    exot::utilities::do_not_optimise(read);
  });
#endif
}

inline __attribute__((always_inline, optimize("-O2"))) std::uint64_t
timed_access_O2(buffer_type& buffer, typename buffer_type::value_type& read,
                int n, int os) {
#if defined(__x86_64__)
  return exot::utilities::timeit<exot::primitives::MemoryFencedTSC>(
      [&]() mutable {
        for (auto i = 0; i < n; ++i) { read += buffer[(1 << os) * i]; }
        exot::utilities::do_not_optimise(read);
      });
#else
  return exot::utilities::default_timing_facility([&]() mutable {
    for (auto i = 0; i < n; ++i) { read += buffer[(1 << os) * i]; }
    exot::utilities::do_not_optimise(read);
  });
#endif
}

inline __attribute__((always_inline, optimize("-O3"))) std::uint64_t
timed_access_O3(buffer_type& buffer, typename buffer_type::value_type& read,
                int n, int os) {
#if defined(__x86_64__)
  return exot::utilities::timeit<exot::primitives::MemoryFencedTSC>(
      [&]() mutable {
        for (auto i = 0; i < n; ++i) { read += buffer[(1 << os) * i]; }
        exot::utilities::do_not_optimise(read);
      });
#else
  return exot::utilities::default_timing_facility([&]() mutable {
    for (auto i = 0; i < n; ++i) { read += buffer[(1 << os) * i]; }
    exot::utilities::do_not_optimise(read);
  });
#endif
}

inline __attribute__((always_inline, optimize("-Og"))) std::uint64_t
timed_access_Og(buffer_type& buffer, typename buffer_type::value_type& read,
                int n, int os) {
#if defined(__x86_64__)
  return exot::utilities::timeit<exot::primitives::MemoryFencedTSC>(
      [&]() mutable {
        for (auto i = 0; i < n; ++i) { read += buffer[(1 << os) * i]; }
        exot::utilities::do_not_optimise(read);
      });
#else
  return exot::utilities::default_timing_facility([&]() mutable {
    for (auto i = 0; i < n; ++i) { read += buffer[(1 << os) * i]; }
    exot::utilities::do_not_optimise(read);
  });
#endif
}

inline __attribute__((always_inline, optimize("-Os"))) std::uint64_t
timed_access_Os(buffer_type& buffer, buffer_type::value_type& read, int n,
                int os) {
#if defined(__x86_64__)
  return exot::utilities::timeit<exot::primitives::MemoryFencedTSC>(
      [&]() mutable {
        for (auto i = 0; i < n; ++i) { read += buffer[(1 << os) * i]; }
        exot::utilities::do_not_optimise(read);
      });
#else
  return exot::utilities::default_timing_facility([&]() mutable {
    for (auto i = 0; i < n; ++i) { read += buffer[(1 << os) * i]; }
    exot::utilities::do_not_optimise(read);
  });
#endif
}

/* The order of variables in the initialiser list matters, and is defined from
 * last to first. */
cache_l1::cache_l1(cache_l1::settings& conf)
    : barrier_(conf_.cores.size() + 1), conf_{::details::bootstrap(conf)} {
  readings_.resize(conf_.cores.size());

#if !defined(__x86_64__)
  /* Call now() for 1-time initialisation of the perf_clock. */
  // exot::primitives::fenced_clock<exot::primitives::perf_clock>::now();
#endif

  auto l1 = (conf_.cache_info.has_value()
                 ? exot::utilities::CPUCacheInfo(0, conf_.cache_info.value())
                 : exot::utilities::CPUCacheInfo(0))
                .at(1, static_cast<int>(exot::utilities::CacheType::Data));

  n_ = static_cast<int>(l1.ways_of_associativity().value());
  o_ = static_cast<int>(std::log2(l1.coherency_line_size().value()));
  s_ = static_cast<int>(std::log2(l1.number_of_sets().value()));
  b_ = static_cast<int>(n_ * (1 << (o_ + s_)));

  debug_log_->info("[cache_l1] using n: {:#x}, o: {:#x}, s: {:#x}, b: {:#x}",
                   n_, o_, s_, b_);

  unsigned idx{0};

  for (auto core : conf_.cores) {
    threads_.emplace_back(
        [idx, core, b = b_, this](barrier_type& barrier) {
          Thread::set_affinity(core);
          Thread::set_scheduling(exot::utilities::SchedulingPolicy::RoundRobin,
                                 90);
          debug_log_->debug("[cache_l1] worker {}: {}", idx,
                            exot::utilities::thread_info());

          buffer_type buffer(b, 0);
          typename buffer_type::value_type read = 0ul;

          static thread_local auto local_n   = n_;
          static thread_local auto local_o_s = (o_ + s_);

          /* At the first barrier, all buffers are allocated and memset'ed. */
          barrier.wait();

          while (flag_.load(std::memory_order_acquire)) {
            /* At the seconds barrier, threads wait for master thread to
             * initiate the measurement. */
            barrier.wait();

            /* Once passed, the timed access is executed. */
            volatile auto current_value =
                timed_access_Os(buffer, read, local_n, local_o_s);

            /* Workers write the current value to the readings vector while
             * holding a lock, to prevent race conditions. Vector writing
             * is/might not be thread-safe. */
            worker_flag_.test_and_set(std::memory_order_acquire);
            readings_.at(idx) = std::move(current_value);
            worker_flag_.clear(std::memory_order_release);

            /* After passing the last barrier values are guaranteed to have been
             * written into the readings vector. */
            barrier.wait();
          }
        },
        std::ref(barrier_));

    ++idx;
  }

  barrier_.wait();
}

cache_l1::~cache_l1() {
  flag_.store(false, std::memory_order_release);

  barrier_.wait();
  barrier_.wait();

  for (auto& thread : threads_) {
    barrier_.force_progress();
    thread.join();
  }
}

typename cache_l1::return_type cache_l1::measure() {
  /* Make progress on workers */
  barrier_.wait();  // measurements are being made
  barrier_.wait();  // measurements are wrtten back

  return readings_;
}

std::vector<std::string> cache_l1::header() {
  std::vector<std::string> descriptions;

  for (auto core : conf_.cores) {
    descriptions.push_back(exot::utilities::generate_header(
        conf_.name(), "access_time", core, "cycles"));
  }

  return descriptions;
}
