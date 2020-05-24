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
 * @file utilities/workers.h
 * @author     Bruno Klopott
 * @brief      Workers for running callable objects.
 */

#pragma once

#include <functional>  // for invoke

#include <fmt/format.h>
#include <spdlog/spdlog.h>  // for logging

#include <exot/framework/state.h>    // for State type
#include <exot/utilities/barrier.h>  // for Barrier
#include <exot/utilities/thread.h>   // for ThreadTraits

namespace exot::utilities {

inline namespace workers {
/**
 * @brief      Basic worker
 *
 * @tparam     Work  A callable type
 * @tparam     Args  Argument types to the callable object
 */
template <class Work, typename... Args>
struct Worker {
  using state_pointer  = std::shared_ptr<exot::framework::State>;
  using logger_pointer = std::shared_ptr<spdlog::logger>;
  using work_type      = Work;

  /**
   * No default constructor
   */
  Worker() = delete;

  /**
   * @brief      Constructor for the basic worker
   *
   * @param[in]  state      The pointer to a state object
   * @param[in]  work       A callable object
   * @param[in]  logger     A pointer to a debug logger
   */
  Worker(state_pointer state, work_type&& work, logger_pointer logger = nullptr)
      : state_(state), work_(std::forward<work_type>(work)){};

  /**
   * @brief      Main operation
   *
   * @param[in]  args  Arguments to the callable object `work_`
   */
  void operator()(Args&&... args) {
#if defined(SPDLOG_TRACE_ON)
    if (logger_ != nullptr)
      logger_->trace("[Worker] {}", exot::utilities::thread_info());
#endif

    /* Repeatedly invoke the work function with its arguments until the state is
     * marked as stopped. */
    while (!state_->is_stopped()) {
      std::invoke(std::forward<work_type>(work_), std::forward<Args>(args)...);
    }

#if defined(SPDLOG_TRACE_ON)
    if (logger_ != nullptr) logger_->trace("[Worker] {}", "exiting");
#endif
  };

 private:
  state_pointer state_;  //! A pointer to a state object
  work_type work_;       //! The callable object invoked in the processing loop
  logger_pointer logger_;  //! A pointer to a debug logger
};

/**
 * @brief      Synchronised worker with barriers
 * @details    The work function is performed between two barriers.
 *
 * @tparam     Work  A callable object of signature void()
 */
template <class Work = std::function<void()>>
struct BarrierWorker {
  using state_pointer  = std::shared_ptr<exot::framework::State>;
  using barrier_type   = exot::utilities::Barrier;
  using logger_pointer = std::shared_ptr<spdlog::logger>;
  using work_type      = Work;

  /**
   * No default constructor
   */
  BarrierWorker() = delete;

  /**
   * @brief      Constructor for the barrier worker
   *
   * @param[in]  state      The pointer to a state object
   * @param      barrier    The reference to a barrier object
   * @param[in]  logger     A pointer to a debug logger
   * @param[in]  work       A callable object
   */
  BarrierWorker(state_pointer state, barrier_type& barrier, work_type&& work,
                logger_pointer logger = nullptr)
      : state_(state), barrier_{barrier}, work_(work), logger_{logger} {};

  /**
   * @brief      Main operation
   *
   * @param[in]  barrier  A reference to a barrier
   */
  void operator()() {
#if defined(SPDLOG_TRACE_ON)
    if (logger_ != nullptr)
      logger_->trace("[BarrierWorker] {}", exot::utilities::thread_info());
#endif

    while (!state_->is_stopped()) {
      barrier_.wait();
      work_();
      barrier_.wait();
    }

#if defined(SPDLOG_TRACE_ON)
    if (logger_ != nullptr) logger_->trace("[BarrierWorker] {}", "exiting");
#endif
  };

 private:
  state_pointer state_;  //! A pointer to a state object
  work_type work_;       //! The callable object invoked in the processing loop
  logger_pointer logger_;  //! A pointer to a debug logger
  barrier_type& barrier_;  //! The barrier object
};

/**
 * @brief      Worker with custom thread properties
 *
 * @tparam     Work  A callable object of signature void()
 */
template <class Work = std::function<void()>>
struct PinnedWorker {
  using state_pointer  = std::shared_ptr<exot::framework::State>;
  using logger_pointer = std::shared_ptr<spdlog::logger>;
  using work_type      = Work;

  /**
   * No default constructor
   */
  PinnedWorker() = delete;

  /**
   * @brief      Constructor for the pinned worker
   *
   * @param[in]  state      The pointer to a state object
   * @param[in]  pin        The core to which the function is to be pinned
   * @param[in]  policy     The scheduling policy of the executing thread
   * @param[in]  priority   The scheduling priority of the executing thread
   * @param[in]  work       A callable object
   * @param[in]  logger     A pointer to a debug logger
   */
  PinnedWorker(state_pointer state, unsigned int pin,
               exot::utilities::SchedulingPolicy policy, unsigned int priority,
               work_type&& work, logger_pointer logger = nullptr)
      : state_(state),
        work_(work),
        pin_(pin),
        policy_(policy),
        priority_(priority),
        logger_(logger){};

  /**
   * @brief      Main operation
   */
  void operator()() {
    /* Set thread affinity and scheduling properties. */
    ThreadTraits::set_affinity(pin_);
    ThreadTraits::set_scheduling(policy_, priority_);

#if defined(SPDLOG_TRACE_ON)
    if (logger_ != nullptr)
      logger_->trace("[PinnedWorker] {}", exot::utilities::thread_info());
#endif

    while (!state_->is_stopped()) { work_(); }

#if defined(SPDLOG_TRACE_ON)
    if (logger_ != nullptr) logger_->trace("[PinnedWorker] {}", "exiting");
#endif
  };

 private:
  state_pointer state_;  //! A pointer to a state object
  work_type work_;       //! The callable object invoked in the processing loop
  logger_pointer logger_;  //! A pointer to a debug logger
  unsigned int pin_;       //! Core to which the executing thread is pinned
  exot::utilities::SchedulingPolicy
      policy_;             //! The scheduling policy of the executing thread
  unsigned int priority_;  //! The scheduling priority of the executing thread
};

/**
 * @brief      Synchronised worker with barriers
 * @details    The work function is performed between two barriers.
 *
 * @tparam     Work  A callable object of signature void()
 */
template <class Work = std::function<void()>>
struct PinnedBarrierWorker {
  using state_pointer  = std::shared_ptr<exot::framework::State>;
  using barrier_type   = exot::utilities::Barrier;
  using logger_pointer = std::shared_ptr<spdlog::logger>;
  using work_type      = Work;

  /* No default constructor */
  PinnedBarrierWorker() = delete;

  /**
   * @brief      Constructor for the pinned barrier worker
   *
   * @param[in]  state      The pointer to a state object
   * @param[in]  pin        The core to which the function is to be pinned
   * @param[in]  policy     The scheduling policy of the executing thread
   * @param[in]  priority   The scheduling priority of the executing thread
   * @param[in]  work       A callable object
   * @param[in]  logger     A pointer to a debug logger
   */
  PinnedBarrierWorker(state_pointer state, unsigned int pin,
                      exot::utilities::SchedulingPolicy policy,
                      unsigned int priority, barrier_type& barrier,
                      work_type&& work, logger_pointer logger = nullptr)
      : state_(state),
        work_(work),
        pin_(pin),
        policy_(policy),
        priority_(priority),
        barrier_{barrier},
        logger_(logger){};

  /**
   * @brief      Main operation
   *
   * @param[in]  barrier  A reference to a barrier
   */
  void operator()() {
    /* Set thread affinity and scheduling properties. */
    ThreadTraits::set_affinity(pin_);
    ThreadTraits::set_scheduling(policy_, priority_);

#if defined(SPDLOG_TRACE_ON)
    if (logger_ != nullptr)
      logger_->trace("[PinnedBarrierWorker] {}",
                     exot::utilities::thread_info());
#endif

    while (!state_->is_stopped()) {
      barrier_.wait();
      work_();
      barrier_.wait();
    }

#if defined(SPDLOG_TRACE_ON)
    if (logger_ != nullptr)
      logger_->trace("[PinnedBarrierWorker] {}", "exiting");
#endif
  };

 private:
  state_pointer state_;  //! A pointer to a state object
  work_type work_;       //! The callable object invoked in the processing loop
  logger_pointer logger_;  //! A pointer to a debug logger
  unsigned int pin_;       //! Core to which the executing thread is pinned
  exot::utilities::SchedulingPolicy
      policy_;             //! The scheduling policy of the executing thread
  unsigned int priority_;  //! The scheduling priority of the executing thread
  barrier_type& barrier_;  //! The barrier object
};

/**
 * The following implementation uses the concept of policy-based design, where
 * policy/mixin classes are used to provide functionality and structure.
 *
 * Classes have internally defined configuration structures, such that the class
 * using the mixins can provide generic constructors. Constructors can be
 * provided with configuration inline, by using initialiser lists `{}`.
 */

inline namespace policy_classes {
/**
 * Mixin classes that provide a function `run()`, which executes a non-returning
 * function with the desired synchronisation method.
 */
inline namespace synchronisation {
/**
 * @brief      Executes a function without any synchronisation.
 */
struct NoSynchronisation {
  struct Configuration {};
  Configuration conf_;

  NoSynchronisation(Configuration& conf) : conf_(conf){};
  virtual ~NoSynchronisation() = default;

  inline void run(std::function<void()>&& f) { f(); };
};

/**
 * @brief      Executes a function between two calls to thread barriers.
 */
struct BarrierSynchronisation {
  struct Configuration {
    exot::utilities::Barrier& barrier;
  };
  Configuration conf_;

  BarrierSynchronisation(Configuration& conf) : conf_(conf){};
  virtual ~BarrierSynchronisation() = default;

  inline void run(std::function<void()>&& f) {
    conf_.barrier.wait();
    f();
    conf_.barrier.wait();
  };
};

}  // namespace synchronisation

/**
 * Mixin classes that allow setting thread parameters prior inside the used
 * thread. They provide an `enforce()` function, which sets thread affinity
 * and/or scheduling.
 */
inline namespace threading {
struct BasicThreads {
  struct Configuration {};
  Configuration conf_;

  BasicThreads(Configuration& conf) : conf_(conf){};
  virtual ~BasicThreads() = default;

  inline void enforce() {}
};

/**
 * @brief Workers with pinned threads and scheduling policies
 *
 */
struct PinnedThreads {
  struct Configuration {
    const unsigned int pin;                          //! Core to pin
    const exot::utilities::SchedulingPolicy policy;  //! Scheduling policy
    const unsigned int priority;                     //! Scheduling priority
  };
  Configuration conf_;

  PinnedThreads(Configuration& conf) : conf_(conf){};
  virtual ~PinnedThreads() = default;

  inline void enforce() {
    ThreadTraits::set_affinity(conf_.pin);
    ThreadTraits::set_scheduling(conf_.policy, conf_.priority);
  };
};

/**
 * @brief Workers with pinned/un-pinned threads and scheduling policies
 *
 */
struct SpecialisedThreads {
  struct Configuration {
    const unsigned int pin;                          //! Core to pin
    const bool should_pin;                           //! Should pin?
    const exot::utilities::SchedulingPolicy policy;  //! Scheduling policy
    const unsigned int priority;                     //! Scheduling priority
  };
  Configuration conf_;

  SpecialisedThreads(Configuration& conf) : conf_(conf){};
  virtual ~SpecialisedThreads() = default;

  inline void enforce() {
    if (conf_.should_pin) ThreadTraits::set_affinity(conf_.pin);
    ThreadTraits::set_scheduling(conf_.policy, conf_.priority);
  };
};

}  // namespace threading

}  // namespace policy_classes

/**
 * @brief      Class for templated worker, which uses policy-based design.
 * @details    Uses one of the mixin classes above to provide additional
 *             functionality.
 *
 * @tparam     SynchronisationPolicy  A synchronisation mixin class type
 * @tparam     ThreadingPolicy        A threading policy mixin class type
 */
template <typename SynchronisationPolicy, typename ThreadingPolicy>
class TemplatedWorker : public SynchronisationPolicy, public ThreadingPolicy {
 public:
  using state_pointer  = std::shared_ptr<exot::framework::State>;
  using logger_pointer = std::shared_ptr<spdlog::logger>;
  using work_type      = std::function<void()>;

  /**
   * @brief      Constructs the templated worker
   *
   * @param[in]  state      A pointer to a state object
   * @param[in]  synch      Synchronisation configuration
   * @param[in]  thread.    Threading configuration
   * @param[in]  work       The callable work object
   * @param[in]  logger     A pointer to a debug logger
   */
  TemplatedWorker(state_pointer state,
                  typename SynchronisationPolicy::Configuration&& synch,
                  typename ThreadingPolicy::Configuration&& thread,
                  work_type&& work, logger_pointer logger = nullptr)
      : state_(state),
        logger_(logger),
        SynchronisationPolicy(synch),
        ThreadingPolicy(thread),
        work_(work){};

  void operator()() {
    ThreadingPolicy::enforce();

#if defined(SPDLOG_TRACE_ON)
    if (logger_ != nullptr)
      logger_->trace("[TemplatedWorker] {}", exot::utilities::thread_info());
#endif

    while (!state_->is_stopped()) {
      SynchronisationPolicy::run(std::forward<work_type>(work_));
    }

#if defined(SPDLOG_TRACE_ON)
    if (logger_ != nullptr) logger_->trace("[TemplatedWorker] {}", "exiting");
#endif
  };

 private:
  work_type work_;
  state_pointer state_;
  logger_pointer logger_;
};

}  // namespace workers

}  // namespace exot::utilities
