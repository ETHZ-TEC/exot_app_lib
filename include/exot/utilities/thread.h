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
 * @file utilities/thread.h
 * @author     Bruno Klopott
 * @brief      Utilities for configuring threads.
 */

#pragma once

#include <string>  // for std::string

/**
 * These headers are available on Linux, Android, and Darwin platforms. If
 * Windows support is desired in the future, a different mechanism for
 * converting SchedulingPolicy enum to its respective platform-specific values
 * will be needed, but will not impact client code.
 */
#include <pthread.h>  // for pthread_t
#include <sched.h>    // for SCHED_*

namespace exot::utilities {

#ifndef SCHED_DEADLINE
#define SCHED_DEADLINE 31
#endif

#ifndef SCHED_BATCH
#define SCHED_BATCH 32
#endif

#ifndef SCHED_IDLE
#define SCHED_IDLE 33
#endif

/**
 * @brief      A typed enum class which holds implementation-specific scheduling
 *             policy values
 * @details    An enum is used so that users do not need to include the
 *             <sched.h> header, and also to make the function calls more
 *             explicit, since functions would take the same type `int` in
 *             direct succession.
 */
enum class SchedulingPolicy : int {
  Other      = SCHED_OTHER,
  Fifo       = SCHED_FIFO,
  RoundRobin = SCHED_RR,
  Deadline   = SCHED_DEADLINE,
  Batch      = SCHED_BATCH,
  Idle       = SCHED_IDLE
};

/**
 * @brief      Produce an info string containing information about the current
 *             thread
 *
 * @return     A string containing the following: the id of the thread, the core
 *             on which the thread is running, the scheduling policy and
 *             priority
 */
std::string thread_info();

/**
 * @brief      A class that holds static methods to set thread properties
 * @details    May have no effect on certain platforms. The functions are
 *             arranged in a class such that it can potentially be used, for
 *             example, as a template parameter, injecting behaviour in other
 *             classes.
 */
struct ThreadTraits {
  /**
   * @brief      Sets the affinity of the current thread
   *
   * @param[in]  cpu   The cpu to which the thread is to be pinned
   */
  static int set_affinity(unsigned int cpu);

  /**
   * @brief      Sets the affinity of the thread via its handle
   * @details    Pinning `std::thread` object can be performed by calling the
   *             `native_handle()` member function.
   *
   * @param[in]  native_handle  The native handle of the thread
   * @param[in]  cpu            The cpu to which the thread is to be pinned
   */
  static int set_affinity(pthread_t native_handle, unsigned int cpu);

  /**
   * @brief      Sets the scheduling policy and priority of the current thread
   *
   * @param[in]  policy    The desired scheduling policy
   * @param[in]  priority  The desired priority
   */
  static int set_scheduling(SchedulingPolicy policy, int priority);

  /**
   * @brief      Sets the scheduling policy and priority of a thread via its
   *             handle
   *
   * @param[in]  native_handle  The native handle of the thread
   * @param[in]  policy         The desired scheduling policy
   * @param[in]  priority       The desired priority
   */
  static int set_scheduling(pthread_t native_handle, SchedulingPolicy policy,
                            int priority);
};

}  // namespace exot::utilities
