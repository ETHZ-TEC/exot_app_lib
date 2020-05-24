/**
 * @file utilities/barrier.h
 * @author     Bruno Klopott
 * @brief      Barrier synchronisation primitive using only the STL.
 */

#pragma once

#include <condition_variable>  // for std::condition_variable
#include <mutex>               // for mutexes
#include <stdexcept>           // for throwing

namespace exot::utilities {

/**
 * @brief      The barrier thread synchronisation primitive, also known as a
 *             randezvous point
 * @details    As each thread arrives at the barrier, it increments a counter,
 *             which states how many threads in total have reached the
 *             randezvous point. Threads lock on a mutex and are descheduled
 *             until the number of threads at the barrier is equal to the
 *             specified count. The thread last to reach the barrier unlocks the
 *             mutex and notifies all waiting threads.
 *
 *             The interface is modelled after barriers found in projects
 *             'boost/thread' and 'boost/fiber'.
 */
class Barrier {
 public:
  using mutex_type = std::mutex;
  using cv_type    = std::condition_variable;

  /**
   * @brief      No default constructor
   */
  Barrier() = delete;

  /**
   * @brief      Constructs the object with desired number of threads that need
   *             to reach the barrier
   *
   * @param[in]  count  The number of threads required for passing the barrier
   */
  explicit Barrier(unsigned int count);

  ~Barrier() = default;

  /**
   * @brief      Waits until the number of threads at the barrier is equal to
   *             the desired count.
   *
   * @return     Last thread to enter the barrier returns true, others return
   *             false.
   */
  bool wait();

  /**
   * @brief Forces progress even if not all threads have reached the barrier
   */
  void force_progress();

 private:
  volatile unsigned int desired_count_;  //! The number of threads that have to
                                         //! reach the randezvous point
  volatile unsigned int current_count_{0};  //! The current number of threads at
                                            //! the randezvous point
  volatile unsigned int repetition_count_{0};  //! The number of times the
                                               //! barrier was reused

  mutex_type mutex_;  //! The mutex used for thread-safe access to variables
  cv_type countdown_reached_cv_;  //! The condition variable used for waiting
                                  //! and notifications
};

}  // namespace exot::utilities
