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
 * @file generators/cache_maurice_mt.cpp
 * @author     Bruno Klopott
 * @brief      Multi-threaded generator causing LLC cache eviction.
 */

#include <exot/generators/cache_maurice_mt.h>

#include <cmath>

#include <spdlog/spdlog.h>

#include <exot/utilities/cache.h>

using namespace exot::modules;

generator_cache_maurice_mt::generator_cache_maurice_mt(
    generator_cache_maurice_mt::settings& conf)
    : bitset_generator_base(conf), lconf_{conf} {
  using namespace exot::utilities;
  using std::log2;

  /* Get information about the cache. */
  auto cache_info = (lconf_.cache_info.has_value()
                         ? CPUCacheInfo(0, lconf_.cache_info.value())
                         : CPUCacheInfo(0));
  auto llc        = cache_info.llc();

  auto llc_associativity  = llc.ways_of_associativity().value();
  auto llc_line_size      = llc.coherency_line_size().value();
  auto llc_number_of_sets = llc.number_of_sets().value();

  logger_->info(
      "[generator->generator_cache_maurice_mt] LLC  associativity: {}, "
      "line size: {}, number of sets: {}",
      llc_associativity, llc_line_size, llc_number_of_sets);

  n_ = static_cast<int>(llc_associativity);         //! LLC ways
  o_ = static_cast<int>(log2(llc_line_size));       //! offset field
  s_ = static_cast<int>(log2(llc_number_of_sets));  //! set field
  c_ = lconf_.overprovision_factor;                 //! buffer factor
  m_ = static_cast<int>(n_ * c_);                   //! buffer multiplier
  b_ = static_cast<int>(m_ * (1 << (o_ + s_)));     //! buffer size

  logger_->info(
      "[generator->generator_cache_maurice_mt] using n: {}, o: {}, s: {}, "
      "c: {:.2f}, m: {}, b: {:#x}",
      n_, o_, s_, c_, m_, b_);
}

void generator_cache_maurice_mt::generate_load(
    const generator_cache_maurice_mt::decomposed_type& decomposed_subtoken,
    const generator_cache_maurice_mt::enable_flag_type& flag,
    generator_cache_maurice_mt::core_type core,
    generator_cache_maurice_mt::index_type index) {
  volatile constexpr static thread_local std::uint8_t constant = 0xAA;

  /* If tracing is enabled, we also track how many buffer traversals were
   * completed. */
#ifdef SPDLOG_TRACE_ON
  auto count = int{0};
#endif

  /* Since all buffers have thread-local storage, we cannot resize/allocate them
   * in the constructor, it's only possible once the worker threads are created.
   *
   * The thread-local "once flags" are used to perform the resizing the first
   * time `generate_load` is called. The resizing is performed in this function
   * because, given the duration of the buffer traversal, the overhead is less
   * impactful then in the `decompose_subtoken` function. Moreover, the latter
   * is performed while holding a lock, therefore has to be kept lightweight.
   */
  std::call_once(this->once_flag_, [this, core] {
    this->buffer_.resize(b_);
    SPDLOG_LOGGER_TRACE(logger_,
                        "[generator->generate_load] resizing buffer for "
                        "core {}: buffer @{} -> {:#x}",
                        core, fmt::ptr(std::addressof(this->buffer_)),
                        this->buffer_.size());
  });

  if (decomposed_subtoken) {
    SPDLOG_LOGGER_TRACE(
        logger_, "[generator->generate_load] worker on core {} enabled", core);

    while (flag) {
      /* Access elements in the buffer. */
      for (auto i = 0; i < (1 << s_); ++i) {
        for (auto j = 0; j < m_; ++j) {
          auto idx              = (1 << o_) * i + (1 << (o_ + s_)) * j;
          this->buffer_.at(idx) = constant;
        }
      }
#ifdef SPDLOG_TRACE_ON
      ++count;
#endif
    }

    SPDLOG_LOGGER_TRACE(logger_,
                        "[generator->generate_load] worker on core {} "
                        "completed {} loops through buffer",
                        core, count);
  }
}
