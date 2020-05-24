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
