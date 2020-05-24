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
 * @file framework/executor.h
 * @author     Bruno Klopott
 * @brief      Executors for running process network nodes and arbitrary
 *             callable objects.
 */

#pragma once

#include <algorithm>    // for remove_if
#include <functional>   // for std::invoke
#include <mutex>        // for mutexes
#include <stdexcept>    // for throwing
#include <thread>       // for threading
#include <type_traits>  // for type traits
#include <vector>       // for holding thread workers

#ifdef EXOT_USE_FIBERS
#include <boost/fiber/all.hpp>
#endif

#include <exot/framework/node.h>    // for IProcess
#include <exot/utilities/thread.h>  // for thread specialisation
#include <exot/utilities/types.h>   // for type traits

#if defined(__GNUC__) && !defined(__clang__)
#include <features.h>
#if __GNUC_PREREQ(8, 0)
#define PROVIDE_VARIADIC_SPAWN
#endif
#endif

namespace exot::framework {

using exot::utilities::SchedulingPolicy;
using exot::utilities::ThreadTraits;

/**
 * @brief      Base class for executors
 * @details    The generic spawning interface cannot be included in the base
 *             class, since function templates cannot be virtual.
 */
class IExecutor {
 public:
  /**
   * @brief      Spawns a node that derives from the IProcess class
   *
   * @param      node  The process network node
   */
  virtual void spawn(IProcess& node) = 0;
  virtual ~IExecutor()               = default;
};

/**
 * @brief      Executor that spawns each callable object in a new thread.
 */
class ThreadExecutor : public virtual IExecutor {
 public:
  ThreadExecutor() = default;

  void spawn(IProcess& obj) override {
    std::lock_guard<std::mutex> lg(threads_mutex_);
    threads_.emplace_back([&]() { obj.process(); });
  }

#if defined(__clang__) || defined(PROVIDE_VARIADIC_SPAWN)

  /**
   * @brief      A template function for spawning multiple process network nodes
   * @details    The template will only be enabled for objects that have a
   *             void(void) process() function, and only if more than 1 argument
   *             is provided, so that in the default case the spawn function
   *             above is used.
   */
  template <typename... T,
            typename = std::enable_if_t<
                (exot::utilities::has_process_function_v<T> && ...)>,
            typename = std::enable_if_t<(sizeof...(T) > 1)>>
  void spawn(T&... objs) {
    std::lock_guard<std::mutex> lg(threads_mutex_);
    (..., threads_.emplace_back([&]() { objs.process(); }));
  }

#endif

  /**
   * @brief      Spawns a callable object, providing it with its arguments.
   *
   * @param[in]  callable  A callable object (can be invoked with std::invoke).
   * @param[in]  args      Arguments passed to the callable object.
   *
   * @tparam     Callable  A callable object type.
   * @tparam     Args      Callable object's argument types.
   */
#if defined(__cpp_lib_is_invocable)
  template <
      typename Callable, typename... Args,
      typename = std::enable_if_t<std::is_invocable<Callable, Args...>::value>>
#else
  template <
      typename Callable, typename... Args,
      typename = std::enable_if_t<std::__invokable<Callable, Args...>::value>>
#endif
  void spawn(Callable&& callable, Args&&... args) {
    std::lock_guard<std::mutex> lg(threads_mutex_);
    /// Emplacing allows constructing the object in place, therefore the thread
    /// object does not need to be constructed manually.
    threads_.emplace_back(std::forward<Callable>(callable), args...);
  };

  /**
   * @brief      Spawns a callable object and assigns thread properties.
   *
   * @param[in]  cpu       CPU to pin the spawned thread to.
   * @param[in]  policy    Scheduling policy of the spawned thread.
   * @param[in]  priority  Scheduling priority of the spawned thread.
   * @param[in]  callable  A callable object (can be invoked with std::invoke).
   * @param[in]  args      Arguments passed to the callable object.
   *
   * @tparam     Callable  A callable object type.
   * @tparam     Args      Callable object's argument types.
   */
  template <typename Callable, typename... Args>
  void spawn_with_properties(unsigned int cpu, SchedulingPolicy policy,
                             unsigned int priority, Callable&& callable,
                             Args&&... args) {
    /* At the moment thread traits only accept arguments passed as values, so
     * the lambda capture required for callable and args... does not behave
     * correctly for simple values. Since arguments are very simple and small
     * types, it is usually not preferred to have them passed as references. */

    /**
     * Before the function 'callable' is invoked the thread properties are set.
     */
    auto function = [&](unsigned int cpu_, SchedulingPolicy policy_,
                        unsigned int priority_) {
      ThreadTraits::set_affinity(cpu_);
      ThreadTraits::set_scheduling(policy_, priority_);
      std::invoke(callable, args...);
    };

    /* A thread is added to the registry */
    {
      std::lock_guard<std::mutex> lg(threads_mutex_);
      threads_.emplace_back(function, cpu, policy, priority);
    }
  };

  /**
   * @brief      Join all threads that belong to the executor.
   * @details    While joining threads in the destructor works most of the
   *             time, in some cases, especially when the callable contains
   *             objects (esp. objects with mutexes) that might be destroyed
   *             during program termination while the spawned thread was
   *             asleep. For example, consider the code:
   *
   * @code{C++}
   * //...
   * ThreadExecutor exec;
   * ObjectWithLocking obj;
   * //...
   * exec.spawn([&obj]{ std::this_thread::sleep_for(10s); obj.use(); });
   * //...
   * @endcode
   *
   *             Local objects are destroyed in reverse order of construction.
   *             Therefore `obj`'s destructor will be called first, followed by
   *             the destructor of `ThreadExecutor`. If the destructor is
   *             called before the spawned thread wakes up, there is a chance
   *             that `obj` will no longer exist. To avoid that, it is safer to
   *             join the executor before exiting from the main routine.
   *
   *             The routine also removes joined threads from the thread
   *             vector, such that the executor can be reused.
   */
  void join() {
    std::lock_guard<std::mutex> lg(threads_mutex_);
    threads_.erase(std::remove_if(threads_.begin(), threads_.end(),
                                  [](auto& thread) -> bool {
                                    if (thread.joinable()) {
                                      thread.join();
                                      return true;
                                    } else {
                                      return false;
                                    }
                                  }),
                   threads_.end());
  };

  /**
   * @brief      Thread safe access to current thread count.
   *
   * @return     Number of working threads.
   */
  typename std::vector<std::thread>::size_type thread_count() {
    std::lock_guard<std::mutex> lg(threads_mutex_);
    return threads_.size();
  };

  /**
   * @brief      Destructor joins held thread if they were not joined manually.
   * @details    No need to acquire the mutex protecting the `threads_` vector,
   *             as destructors are not executed concurrently.
   */
  ~ThreadExecutor() {
    for (auto&& thread : threads_) {
      if (thread.joinable()) thread.join();
    }
  };

 private:
  std::mutex threads_mutex_;  //! Protects the vector of threads.
  std::vector<std::thread> threads_;
};  // namespace exot::framework

#ifdef EXOT_USE_FIBERS

inline namespace fibers {
/**
 * @brief      Executor that spawns each callable object in a new fiber
 *             (userspace thread).
 * @details    The userland threads need to be provided at least one worker to
 *             allow execution.
 *
 * @tparam     WorkAlgorithm  Work algorithm used by the fiber scheduler, e.g.
 *                            work sharing, or round robin.
 */
template <typename WorkAlgorithm = boost::fibers::algo::round_robin>
class FiberExecutor : public virtual IExecutor {
 public:
  using work_algorithm = WorkAlgorithm;

  FiberExecutor() = default;

  void spawn(IProcess& obj) override {
    if (thread_count() == 0)
      throw std::logic_error(
          "no worker threads were present when a fiber was spawned");

    std::lock_guard<std::mutex> lg(worker_fibers_mutex_);
    worker_fibers_.emplace_back([&]() { obj.process(); });
  }

#if defined(__clang__) || defined(PROVIDE_VARIADIC_SPAWN)

  template <typename... T,
            typename = std::enable_if_t<
                (exot::utilities::has_process_function_v<T> && ...)>,
            typename = std::enable_if_t<(sizeof...(T) > 1)>>
  void spawn(T&... processes) {
    if (thread_count() == 0)
      throw std::logic_error(
          "no worker threads were present when a fiber was spawned");

    std::lock_guard<std::mutex> lg(worker_fibers_mutex_);
    (..., worker_fibers_.emplace_back([&]() { objs.process(); }));
  }

#endif

#if defined(__cpp_lib_is_invocable)
  template <
      typename Callable, typename... Args,
      typename = std::enable_if_t<std::is_invocable<Callable, Args...>::value>>
#else
  template <
      typename Callable, typename... Args,
      typename = std::enable_if_t<std::__invokable<Callable, Args...>::value>>
#endif
  void spawn(Callable&& callable, Args&&... args) {
    if (thread_count() == 0)
      throw std::logic_error(
          "no worker threads were present when a fiber was spawned");

    std::lock_guard<std::mutex> lg(worker_fibers_mutex_);
    worker_fibers_.emplace_back(callable, args...);
  };

  /**
   * @brief      Adds a worker thread for the fibers 'pool'. */
  void add_worker_thread() {
    std::lock_guard<std::mutex> lg(worker_threads_mutex_);
    worker_threads_.emplace_back(
        [] { boost::fibers::use_scheduling_algorithm<work_algorithm>(); });
  };

  /**
   * @brief      Registers calling thread as a fiber pool worker. */
  inline void register_as_worker() {
    boost::fibers::use_scheduling_algorithm<work_algorithm>();
  };

  /**
   * @brief      Joins all userland threads belonging to this executor
   */
  void join_fibers() {
    std::lock_guard<std::mutex> lg(worker_fibers_mutex_);
    worker_fibers_.erase(
        std::remove_if(worker_fibers_.begin(), worker_fibers_.end(),
                       [](auto& fiber) -> bool {
                         if (fiber.joinable()) {
                           fiber.join();
                           return true;
                         } else {
                           return false;
                         }
                       }),
        worker_fibers_.end());
  }

  /**
   * @brief      Joins all worker threads belonging to this executor
   */
  void join_workers() {
    std::lock_guard<std::mutex> lg(worker_threads_mutex_);
    worker_threads_.erase(
        std::remove_if(worker_threads_.begin(), worker_threads_.end(),
                       [](auto& thread) -> bool {
                         if (thread.joinable()) {
                           thread.join();
                           return true;
                         } else {
                           return false;
                         }
                       }),
        worker_threads_.end());
  }

  /**
   * @brief      Thread safe access to current thread count.
   *
   * @return     Number of working worker threads.
   */
  typename std::vector<std::thread>::size_type thread_count() {
    std::lock_guard<std::mutex> lg(worker_threads_mutex_);
    return worker_threads_.size();
  };

  /**
   * @brief      Thread safe access to current fiber count.
   *
   * @return     Number of working fiber threads.
   */
  typename std::vector<std::thread>::size_type fiber_count() {
    std::lock_guard<std::mutex> lg(worker_fibers_mutex_);
    return worker_fibers_.size();
  };

  ~FiberExecutor() {
    for (auto&& fiber : worker_fibers_) {
      if (fiber.joinable()) fiber.join();
    }

    for (auto&& thread : worker_threads_) {
      if (thread.joinable()) thread.join();
    }
  };

 private:
  std::mutex worker_threads_mutex_;  //! Protects the vector of worker threads.
  std::vector<std::thread> worker_threads_;

  std::mutex worker_fibers_mutex_;  //! Protects the vector of spawned fibers.
  std::vector<boost::fibers::fiber> worker_fibers_;
};

}  // namespace fibers

#endif

}  // namespace exot::framework
