/**
 * @file framework/queue.h
 * @author     Bruno Klopott
 * @brief      Thread-safe queues with and without timeouts, used for
 *             communicating between process network nodes.
 */

#pragma once

#include <chrono>              // for time_point and duration
#include <condition_variable>  // for std::condition_variable
#include <limits>              // for getting upper capacity limit
#include <mutex>               // for mutexes
#include <queue>               // for underlying queue container

/**
 * For debugging with a console
 */
#ifdef DEBUG_INTERACTIVE
#include <fmt/format.h>  // for formatting strings

/**
 * @brief      Print debugging info
 *
 * @param[in]  function  Pass the __FUNCTION__ via this parameter
 * @param[in]  message   The message to print
 */
inline void queue_debug_print(const char* function, const char* message) {
  fmt::print("[queue] [{}] {}\n", function, message);
}

#define DEBUG_queue(function, message) queue_debug_print(function, message)
#else
#define DEBUG_queue(function, message)
#endif

namespace exot::framework {

/**
 * @brief      Thread-safe bounded queue
 * @details    The queue provides a thread-safe queue suitable for concurrent
 *             producer-consumer application scenarios. The queue utilises two
 *             condition variables, which allow threads operating on the queue
 *             to efficiently wait on empty/full queue.
 *
 *             Threads pushing values onto the queue wait until the queue is
 *             writeable, and notify threads waiting to pop a value.
 *
 *             Threads popping values from the queue wait if the queue is empty,
 *             and notify threads waiting on a full queue.
 *
 *             By default the queue is effectively unbounded, with the default
 *             capacity being the upper limit of the system-specific `size_t`
 *             type.
 *
 * @tparam     Token  Data type stored in the queue.
 * @tparam     Mutex  Mutex type
 * @tparam     CV     Condition variable type
 */
template <typename Token, typename Mutex = std::mutex,
          typename CV = std::condition_variable>
class LockingQueue {
 public:
  using value_type      = Token;
  using reference       = value_type&;
  using const_reference = const value_type&;
  using size_type       = size_t;
  using container       = std::queue<value_type>;
  using mutex_type      = Mutex;
  using cv_type         = CV;

  static const size_type default_capacity{
      std::numeric_limits<size_type>::max()};

  LockingQueue(size_type capacity) : capacity_{capacity} {};
  LockingQueue() : LockingQueue(default_capacity){};

  /**
   * Copy operations/constructors not required and/or not meaningful.
   */
  LockingQueue(const LockingQueue& other) = delete;
  LockingQueue(LockingQueue&& other)      = default;
  LockingQueue& operator=(const LockingQueue& other) = delete;
  LockingQueue& operator=(LockingQueue&& other) = default;

  /**
   * Virtual destructor necessary if the class is going to be subclassed.
   */
  virtual ~LockingQueue() = default;

  /**
   * @brief      Locking wrapped for front()
   *
   * @return     A reference to the front item in the queue.
   */
  reference front() {
    /**
     * @brief      Unique lock in conjunction with a condition variable allow
     *             blocking until a token is available in the queue. */
    std::unique_lock<mutex_type> lock(queue_mutex_);
    /* The wait statement in the CV is somewhat equivalent to
     *
     * @code{C++}
     * while(!predicate()) {
     *   lock.lock();
     * }
     * @endcode
     *
     * The CV will be notified when a value is pushed onto the queue. */
    queue_empty_cv_.wait(lock, [this] { return !empty_(); });
    return queue_.front();
  }

  /**
   * @brief      Locking wrapper for pop()
   */
  void pop() {
    {
      std::unique_lock<mutex_type> lock(queue_mutex_);
      /// Cannot pop an empty queue.
      DEBUG_queue(__FUNCTION__, "locked, waiting on empty ...");
      queue_empty_cv_.wait(lock, [this] { return !empty_(); });
      DEBUG_queue(__FUNCTION__, "was notified or condition set, empty...");
      queue_.pop();
    }

    /* Send a 'notification' via the CV to those waiting on a full queue. */
    DEBUG_queue(__FUNCTION__, "will notify waiting on full!");
    notify_waiting_on_full_();
  }

  /**
   * @brief      Locking wrapper for push() (via reference)
   *
   * @param[in]  value  Token to push onto the queue.
   */
  void push(const_reference value) {
    {
      std::unique_lock<mutex_type> lock(queue_mutex_);
      DEBUG_queue(__FUNCTION__, "locked, waiting on full...");
      queue_full_cv_.wait(lock, [this] { return !full_(); });
      DEBUG_queue(__FUNCTION__, "was notified or condition set...");
      queue_.push(value);
    }

    /* Send a 'notification' via the CV to those waiting on an empty queue. */
    DEBUG_queue(__FUNCTION__, "will notify waiting on empty!");
    notify_waiting_on_empty_();
  }

  /**
   * @brief      Locking wrapper for push() (via move)
   *
   * @param[in]  value  Token to push onto the queue.
   */
  void push(value_type&& value) {
    {
      std::unique_lock<mutex_type> lock(queue_mutex_);
      DEBUG_queue(__FUNCTION__, "locked, waiting on full...");
      queue_full_cv_.wait(lock, [this] { return !full_(); });
      DEBUG_queue(__FUNCTION__, "was notified or condition set...");
      queue_.push(std::forward<decltype(value)>(value));
    }

    /* Send a 'notification' via the CV to those waiting on an empty queue. */
    DEBUG_queue(__FUNCTION__, "will notify waiting on empty!");
    notify_waiting_on_empty_();
  }

  /**
   * @brief      Locking wrapper for emplace().
   * @details    Constructs an element in place, without copying/moving.
   *
   * @tparam     Args  Forwarded arguments to the constructor of Token. */
  template <class... Args>
  decltype(auto) emplace(Args&&... args) {
    {
      std::unique_lock<mutex_type> lock(queue_mutex_);
      DEBUG_queue(__FUNCTION__, "locked, waiting on full...");
      queue_full_cv_.wait(lock, [this] { return !full_(); });
      DEBUG_queue(__FUNCTION__, "was notified or condition set...");
      queue_.emplace(std::forward<Args>(args)...);
    }
    DEBUG_queue(__FUNCTION__, "will notify waiting on empty!");
    notify_waiting_on_empty_();
  }

  /**
   * @brief      Locking wrapper for empty()
   * @details    The attribute specified `[[nodiscard]]` instructs the compiler
   *             to issue a warning if the function's return value is used in a
   *             discarded-value expression (C++ Standard [dcl.attr.nodiscard]
   *             "10.6.8 Nodiscard attribute"). Essentially it means that the
   *             function's return value should not be ignored.
   *
   *             The function has no side effects.
   *
   * @return     Returns true if empty.
   */
  [[nodiscard]] bool empty() {
    std::lock_guard<mutex_type> lg(queue_mutex_);
    return empty_();
  }

  /**
   * @brief      Checks if queue is full.
   * @details    The function has no side effects.
   *
   * @return     Returns true if full
   */
  [[nodiscard]] bool full() {
    std::lock_guard<mutex_type> lg(queue_mutex_);
    return full_();
  }

  /**
   * @brief      Locking wrapper for size()
   * @details    In concurrent queue access the size is only approximate,
   *             unless a mechanism is provided to lock the queue for after
   *             the size() operation.
   *
   * @return     Number of elements in the queue.
   */
  size_type size() {
    std::lock_guard<mutex_type> lg(queue_mutex_);
    return queue_.size();
  }

  /**
   * @brief      Returns the capacity of the queue.
   *
   * @return     The capacity of the queue.
   */
  size_type capacity() const { return capacity_; }

 protected:
  container queue_;         //! Underlying queue container
  mutex_type queue_mutex_;  //! Mutex protecting operations on the container
  cv_type queue_empty_cv_;  //! Condition variable for waiting on empty
  cv_type queue_full_cv_;   //! Condition variable for waiting on full
  size_type capacity_;      //! Capacity of the queue

  /**
   * @brief      Non-thread-safe check if underlying container is equal or
   *             exceeds the desired capacity
   *
   * @return     True if underlying container's size is less than capacity
   */
  inline bool full_() const { return queue_.size() >= capacity_; }
  /**
   * @brief      Non-thread-safe check if underlying container is empty.
   *
   * @return     True if underlying container is empty
   */
  inline bool empty_() const { return queue_.empty(); }

  /**
   * @brief      Wrapper for nofitying threads waiting on a full queue
   */
  inline void notify_waiting_on_full_() { queue_full_cv_.notify_one(); }
  /**
   * @brief      Wrapper for notifying threads waiting on an empty queue
   */
  inline void notify_waiting_on_empty_() { queue_empty_cv_.notify_one(); }
};

/**
 * @brief      Concurrent locking queue with timeout functionality
 * @details    Extends the locking queue with try_{pop,push}_{,for,until}
 *             operations, which provide extended functionality and new
 *             opportunities for structuring program flow and preventing
 *             deadlocks.
 *
 * @tparam     Token  Data type stored in the queue.
 * @tparam     Mutex  Mutex type used for locking.
 * @tparam     CV     Condition variable type used for notifying.
 */
template <typename Token, typename Mutex = std::mutex,
          typename CV = std::condition_variable>
class TimeoutLockingQueue : public LockingQueue<Token, Mutex, CV> {
 public:
  /**
   * Since base class is a class template, typedefs cannot be simply
   * 'inherited'. For details, see C++ standard [temp.dep]/3 ("17.7.2 Dependent
   * names"), which states that in "the definition of a class or class template,
   * the scope of a dependent base class is not examined during unqualified name
   * lookup either at the point of definition of the class template or member or
   * during an instantiation of the class template or member."
   */
  using value_type = typename LockingQueue<Token, Mutex, CV>::value_type;
  using reference  = typename LockingQueue<Token, Mutex, CV>::reference;
  using const_reference =
      typename LockingQueue<Token, Mutex, CV>::const_reference;
  using size_type  = typename LockingQueue<Token, Mutex, CV>::size_type;
  using container  = typename LockingQueue<Token, Mutex, CV>::container;
  using mutex_type = typename LockingQueue<Token, Mutex, CV>::mutex_type;
  using cv_type    = typename LockingQueue<Token, Mutex, CV>::cv_type;

  /**
   * Use the base class' constructor.
   */
  using LockingQueue<Token, Mutex, CV>::LockingQueue;

  /**
   * Since base class is a class template, also the protected members cannot be
   * directly accessed as in the case of simple inheritance.
   */
  using LockingQueue<Token, Mutex, CV>::queue_;
  using LockingQueue<Token, Mutex, CV>::queue_mutex_;
  using LockingQueue<Token, Mutex, CV>::queue_empty_cv_;
  using LockingQueue<Token, Mutex, CV>::queue_full_cv_;

  /**
   * Same declarations are necessary for base class member functions.
   */
  using LockingQueue<Token, Mutex, CV>::push;
  using LockingQueue<Token, Mutex, CV>::full_;
  using LockingQueue<Token, Mutex, CV>::empty_;
  using LockingQueue<Token, Mutex, CV>::notify_waiting_on_empty_;
  using LockingQueue<Token, Mutex, CV>::notify_waiting_on_full_;

  virtual ~TimeoutLockingQueue() = default;

  /**
   * Copy and move operations/constructors not required and not /
   * meaningful. */
  TimeoutLockingQueue(const TimeoutLockingQueue& other)     = delete;
  TimeoutLockingQueue(TimeoutLockingQueue&& other) noexcept = delete;
  TimeoutLockingQueue& operator=(const TimeoutLockingQueue& other) = delete;
  TimeoutLockingQueue& operator=(TimeoutLockingQueue&& other) = delete;

  /**
   * @brief      Pop a value if the queue is not empty
   * @details    Unlike `pop()`, this function will never block on an empty
   *             queue.
   *
   *             This function is preferred to the 'traditional' sequence of
   *             operations `front()` and `pop()`, bacause it does not require
   *             the lock to be held twice.
   *
   *             If the queue is empty, the function has no side effects, i.e.
   *             it will not modify the underlying container.
   *
   * @param      value  The value to pop
   *
   * @return     true if succeeded, false if queue is empty.
   */
  bool try_pop(reference value) {
    {
      std::lock_guard<mutex_type> lg(queue_mutex_);
      if (empty_()) { return false; }

      value = queue_.front();
      queue_.pop();
      /* Lock guard destroyed after leaving this scope, releasing the lock. */
    }
    DEBUG_queue(__FUNCTION__, "will notify waiting on full!");
    notify_waiting_on_full_();
    return true;
  }

  /**
   * @brief      Tries to pop a value with a timeout
   * @details    This and the following functions use the functionality provided
   *             by the condition variable, which allows waiting on the lock
   *             only for a specified duration.
   *
   * @param      value    The value to pop
   * @param[in]  timeout  Timeout duration
   *
   * @tparam     Rep      Storage type used for std::chrono::duration
   * @tparam     Period   Period type used for std::chrono::duration
   *                      (std::ratio)
   *
   * @return     true if could pop within the timeout duration, otherwise false.
   */
  template <typename Rep, typename Period>
  bool try_pop_for(reference value,
                   const std::chrono::duration<Rep, Period>& timeout) {
    std::unique_lock<mutex_type> lock(queue_mutex_);
    DEBUG_queue(__FUNCTION__, "locked, waiting on empty...");
    /* Wait on an empty queue. */
    if (queue_empty_cv_.wait_for(lock, timeout, [this] { return !empty_(); })) {
      value = queue_.front();
      queue_.pop();
      /* Release the lock before notifying waiting threads. */
      lock.unlock();
      DEBUG_queue(__FUNCTION__, "will notify waiting on full!");
      notify_waiting_on_full_();
      return true;
    } else {
      DEBUG_queue(__FUNCTION__, "timed out!");
      return false;
    }
  }

  /**
   * @brief      Tries to pop a value until some time point is reached
   *
   * @param      value     The value to pop
   * @param[in]  time      Timeout time point
   *
   * @tparam     Clock     Clock type used for the time point
   * @tparam     Duration  Duration type used for the time point
   *
   * @return     true if could pop before the time point, otherwise false.
   */
  template <typename Clock, typename Duration>
  bool try_pop_until(reference value,
                     const std::chrono::time_point<Clock, Duration>& time) {
    std::unique_lock<mutex_type> lock(queue_mutex_);
    DEBUG_queue(__FUNCTION__, "locked, waiting on empty...");
    if (queue_empty_cv_.wait_until(lock, time, [this] { return !empty_(); })) {
      DEBUG_queue(__FUNCTION__, "was notified or condition set...");
      value = queue_.front();
      queue_.pop();
      lock.unlock();
      DEBUG_queue(__FUNCTION__, "will notify waiting on full!");
      notify_waiting_on_full_();
      return true;
    } else {
      DEBUG_queue(__FUNCTION__, "timed out!");
      return false;
    }
  }

  /**
   * Push operations via const reference
   */

  /**
   * @brief      Tries to push a value unless the queue is full
   *
   * @param[in]  value  The value to push
   *
   * @return     true if queue was not full and the value was pushed, false
   *             otherwise.
   */
  inline bool try_push(const_reference value) {
    {
      std::lock_guard<mutex_type> lg(queue_mutex_);
      if (full_()) { return false; }

      queue_.push(value);
    }
    notify_waiting_on_empty_();
    return true;
  }

  /**
   * @brief      Tries to push a value with a timeout
   *
   * @param[in]  value    The value to push
   * @param[in]  timeout  The timeout duration
   *
   * @tparam     Rep      Storage type used for std::chrono::duration
   * @tparam     Period   Period type used for std::chrono::duration
   *                      (std::ratio)
   *
   * @return     true if could push within the timeout duration, otherwise
   *             false.
   */
  template <typename Rep, typename Period>
  inline bool try_push_for(const_reference value,
                           const std::chrono::duration<Rep, Period>& timeout) {
    std::unique_lock<mutex_type> lock(queue_mutex_);
    DEBUG_queue(__FUNCTION__, "locked, waiting on full...");
    if (queue_full_cv_.wait_for(lock, timeout, [this] { return !full_(); })) {
      DEBUG_queue(__FUNCTION__, "was notified or condition set,...");
      queue_.push(value);
      lock.unlock();
      DEBUG_queue(__FUNCTION__, "will notify waiting on empty!");
      notify_waiting_on_empty_();
      return true;
    } else {
      DEBUG_queue(__FUNCTION__, "timed out!");
      return false;
    }
  }

  /**
   * @brief      Tries to push a value until some time point is reached
   *
   * @param      value     The value to push
   * @param[in]  time      Timeout time point
   *
   * @tparam     Clock     Clock type used for the time point
   * @tparam     Duration  Duration type used for the time point
   *
   * @return     true if could push before the time point, otherwise false.
   */
  template <typename Clock, typename Duration>
  inline bool try_push_until(
      const_reference value,
      const std::chrono::time_point<Clock, Duration>& time) {
    std::unique_lock<mutex_type> lock(queue_mutex_);
    DEBUG_queue(__FUNCTION__, "locked, waiting on full...");
    if (queue_full_cv_.wait_until(lock, time, [this] { return !full_(); })) {
      DEBUG_queue(__FUNCTION__, "was notified or condition set...");
      queue_.push(value);
      lock.unlock();
      DEBUG_queue(__FUNCTION__, "will notify waiting on empty!");
      notify_waiting_on_empty_();
      return true;
    } else {
      DEBUG_queue(__FUNCTION__, "timed out!");
      return false;
    }
  }

  /**
   * Push operations via move
   */

  /**
   * @brief      Tries to push a value unless the queue is full via a move
   *             operation
   *
   * @param[in]  value  The value to push
   *
   * @return     true if queue was not full and the value was pushed, false
   *             otherwise.
   */
  inline bool try_push(value_type&& value) {
    {
      std::lock_guard<mutex_type> lg(queue_mutex_);
      if (full_()) { return false; }
      queue_.push(std::move(value));
    }
    DEBUG_queue(__FUNCTION__, "will notify waiting on empty!");
    notify_waiting_on_empty_();
    return true;
  }

  /**
   * @brief      Tries to push a value with a timeout via a move operation
   *
   * @param[in]  value    The value to push
   * @param[in]  timeout  The timeout duration
   *
   * @tparam     Rep      Storage type used for std::chrono::duration
   * @tparam     Period   Period type used for std::chrono::duration
   *                      (std::ratio)
   *
   * @return     true if could push within the timeout duration, otherwise
   *             false.
   */
  template <typename Rep, typename Period>
  inline bool try_push_for(value_type&& value,
                           const std::chrono::duration<Rep, Period>& timeout) {
    std::unique_lock<mutex_type> lock(queue_mutex_);
    DEBUG_queue(__FUNCTION__, "locked, waiting on full...");
    if (queue_full_cv_.wait_for(lock, timeout, [this] { return !full_(); })) {
      DEBUG_queue(__FUNCTION__, "was notified or condition set...");
      queue_.push(std::move(value));
      lock.unlock();
      DEBUG_queue(__FUNCTION__, "will notify waiting on empty!");
      notify_waiting_on_empty_();
      return true;
    } else {
      DEBUG_queue(__FUNCTION__, "timed out!");
      return false;
    }
  }

  /**
   * @brief      Tries to push a value until some time point is reached via a
   *             move operation
   *
   * @param[in]  time       Timeout time point
   * @param      value      The value to push
   *
   * @tparam     Clock      Clock type used for the time point
   * @tparam     Duration   Duration type used for the time point
   *
   * @return     true if could push before the time point, otherwise false.
   */
  template <typename Clock, typename Duration>
  inline bool try_push_until(
      value_type&& value,
      const std::chrono::time_point<Clock, Duration>& time) {
    std::unique_lock<mutex_type> lock(queue_mutex_);
    DEBUG_queue(__FUNCTION__, "locked, waiting on full...");
    if (queue_full_cv_.wait_until(lock, time, [this] { return !full_(); })) {
      DEBUG_queue(__FUNCTION__, "was notified or condition set...");
      queue_.push(std::move(value));
      lock.unlock();
      DEBUG_queue(__FUNCTION__, "will notify waiting on empty!");
      notify_waiting_on_empty_();
      return true;
    } else {
      DEBUG_queue(__FUNCTION__, "timed out!");
      return false;
    }
  }
};

#ifdef EXOT_USE_FIBERS

#include <boost/fiber/all.hpp>

/**
 * Since the node interface type accepts the container as a template-template
 * parameter, these aliases are required, because the only type passed to the
 * template-template parameter is the token type.
 *
 * The aliases below use the userspace thread mutexes and condition variables,
 * instead of those in the STL.
 */

template <typename Token>
using FiberLockingQueue = LockingQueue<Token, boost::fibers::mutex,
                                       boost::fibers::condition_variable>;

template <typename Token>
using FiberTimeoutLockingQueue =
    TimeoutLockingQueue<Token, boost::fibers::mutex,
                        boost::fibers::condition_variable>;

#endif

}  // namespace exot::framework
