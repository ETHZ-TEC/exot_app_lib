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
