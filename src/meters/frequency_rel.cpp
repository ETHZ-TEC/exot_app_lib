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
 * @file meters/frequency_rel.cpp
 * @author     Bruno Klopott
 * @brief      Implementation of the empirical frequency metering module.
 */

#include <exot/meters/frequency_rel.h>

#include <type_traits>

#include <fmt/ostream.h>

#include <exot/utilities/configuration.h>
#include <exot/utilities/ostream.h>
#include <exot/utilities/thread.h>

using Thread = exot::utilities::ThreadTraits;

using namespace exot::modules;
using namespace std::chrono_literals;

namespace details {

// const unsigned std_iter = 1024;
// const unsigned std_iter = 2048;
const unsigned std_iter = 4096;

/**
 * Helper to bootstrap cores in the constructor initialiser list, needed for
 * initialising the barrier, which copy/assignment constructors are deleted.
 */
frequency_rel::settings bootstrap(frequency_rel::settings& conf) {
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

/* The order of variables in the initialiser list matters, and is defined from
 * last to first. */
frequency_rel::frequency_rel(frequency_rel::settings& conf)
    : barrier_(conf_.cores.size() + 1), conf_{::details::bootstrap(conf)} {
  readings_.resize(conf_.cores.size());
  references_.resize(conf_.cores.size());

  unsigned idx{0};

  /* Spawn threads and obtain the reference probe durations */
  for (auto core : conf_.cores) {
    threads_.emplace_back(
        [idx, core, this](barrier_type& barrier) {
          Thread::set_affinity(core);
          Thread::set_scheduling(exot::utilities::SchedulingPolicy::RoundRobin,
                                 90);
          debug_log_->debug("[frequency_rel] worker {}: {}", idx,
                            exot::utilities::thread_info());

          {
            worker_flag_.test_and_set(std::memory_order_acquire);
            references_.at(idx) = reference_probe_duration(10);
            worker_flag_.clear(std::memory_order_release);
          }

          barrier.wait();

          while (!flag_.load(std::memory_order_acquire)) {
            barrier.wait();
            {
              auto current_value =
                  references_.at(idx) /
                  details::probe<probe_type, probe_iterations>();
              worker_flag_.test_and_set(std::memory_order_acquire);
              readings_.at(idx) = std::move(current_value);
              worker_flag_.clear(std::memory_order_release);
            }
            barrier.wait();
          }
        },
        std::ref(barrier_));

    ++idx;
  }

  barrier_.wait();
  debug_log_->info("[frequency_rel] reference probes: {}", references_);
}

frequency_rel::~frequency_rel() {
  /* Set termination flag to true and go past barriers. */
  flag_.store(true, std::memory_order_release);

  barrier_.wait();
  barrier_.wait();

  for (auto& thread : threads_) {
    barrier_.force_progress(); /* force progress just in case */
    thread.join();
  }
}

typename frequency_rel::duration_type frequency_rel::reference_probe_duration(
    unsigned repetitions) {
  typename frequency_rel::duration_type reference{0};

  for (unsigned i{0}; i < repetitions; ++i) {
    reference += details::probe<probe_type, probe_iterations>();
    std::this_thread::sleep_for(10ms);
  }

  return (reference / repetitions);
}

typename frequency_rel::return_type frequency_rel::measure() {
  /* Make progress on workers */
  barrier_.wait();
  barrier_.wait();

  return readings_;
}

std::vector<std::string> frequency_rel::header() {
  std::vector<std::string> descriptions;

  for (auto core : conf_.cores) {
    descriptions.push_back(
        exot::utilities::generate_header(conf_.name(), "empirical", core, ""));
  }

  return descriptions;
}
