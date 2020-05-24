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
 * @file utilities/barrier.cpp
 * @author     Bruno Klopott
 * @brief      Implementation of the barrier synchronisation primitive from @ref
 *             barrier.h.
 */

#include <exot/utilities/barrier.h>

namespace exot::utilities {

Barrier::Barrier(unsigned int count) : desired_count_{count} {
  if (count == 0) {
    throw std::logic_error("Barrier count should be larger than zero.");
  }
};

bool Barrier::wait() {
  /* Unique lock type is used in conjunction with a condition variable. */
  std::unique_lock<mutex_type> lock{mutex_};

  /* Since the barrier can be reused multiple times, the repetition count is
   * used to determine the changes. Aliasing does not cause any issues. */
  auto repetition_count = repetition_count_;

  /**
   * Increment the current count as the thread reaches the barrier
   */
  if (++current_count_ == desired_count_) {
    /* If the current count matches the desired count, increment the
     * repetition count, reset the current counter... */
    repetition_count_++;
    current_count_ = 0;

    /* ... unlock the mutex and notify all waiting threads. */
    lock.unlock();
    countdown_reached_cv_.notify_all();

    return true;
  } else {
    /* Otherwise wait until the repetition count is incremented in the block
     * above. */
    countdown_reached_cv_.wait(
        lock, [&, this] { return repetition_count != repetition_count_; });

    return false;
  }
}

void Barrier::force_progress() {
  {
    std::lock_guard<mutex_type> lg{mutex_};

    current_count_ = 0;
    ++repetition_count_;
  }

  countdown_reached_cv_.notify_all();
}

}  // namespace exot::utilities
