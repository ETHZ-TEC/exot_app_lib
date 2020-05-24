/**
 * @file primitives/perf_clock.h
 * @author     Bruno Klopott
 * @brief      Clock source using hardware and software performance counters.
 */

#pragma once

#include <assert.h>
#include <fcntl.h>
#include <linux/perf_event.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>

namespace exot::primitives {

/**
 * @brief   Checks if a number is a valid file descriptor
 *
 * @param   fd     The file descriptor
 * @return  true   The number represents a valid file descriptor
 * @return  false  Otherwise.
 */
inline bool is_fd(long fd) {
  return ::fcntl(fd, F_GETFD) != -1 || errno != EBADF;
}

/**
 * @brief   A custom deleter for file descriptors to be used with unique
 * pointers
 */
struct fd_deleter {
  fd_deleter()             = default;
  fd_deleter(fd_deleter&&) = default;

  void operator()(long* fd) const {
    if (is_fd(*fd)) { ::close(*fd); }
    delete fd;
  }
};

/**
 * @brief   A unique pointer to a file descriptor with a deleter/closer.
 */
using fd_ptr_t = std::unique_ptr<long, fd_deleter>;

/**
 * @brief   A clock source using hardware CPU cycles accessed via Linux'es
 *          performance counters.
 */
class perf_clock {
 public:
  typedef std::chrono::nanoseconds duration;
  typedef duration::rep rep;
  typedef duration::period period;
  typedef std::chrono::time_point<perf_clock> time_point;
  static constexpr bool is_steady = true;
  inline static fd_ptr_t fd_ptr_  = nullptr;

  /**
   * @brief   Reads a value from the performance events file descriptor
   *
   * @return  duration::rep  The hardware CPU cycles
   */
  static duration::rep raw() noexcept {
    long long value;

    if (perf_clock::fd_ptr_) {
      return ::read(*perf_clock::fd_ptr_, &value, sizeof(value)) < sizeof(value)
                 ? duration::rep{0}
                 : static_cast<duration::rep>(value);
    } else {
      return duration::rep{0};
    }
  }

  static time_point now() noexcept {
    static auto init  = perf_clock::init();
    static auto reset = perf_clock::reset();

    auto _ = raw();
    return time_point{duration{_}};
  }

  /**
   * @brief Initialises the access to performance counters.
   */
  static bool init() {
    static struct ::perf_event_attr _;
    _.type                     = ::PERF_TYPE_HARDWARE;
    _.config                   = ::PERF_COUNT_HW_CPU_CYCLES;
    _.size                     = sizeof(_);
    _.exclude_kernel           = 1;
    _.exclude_hv               = 1;
    _.exclude_callchain_kernel = 1;

    // Handle the creation/destruction of the file descriptor to performance
    // events.
    if (perf_clock::fd_ptr_) perf_clock::fd_ptr_.reset(nullptr);
    perf_clock::fd_ptr_.reset(
        new long{::syscall(__NR_perf_event_open, &_, 0, -1, -1, 0)});

    return (*perf_clock::fd_ptr_ >= 0);
  }

  /**
   * @brief Resets access to performance counters.
   */
  static bool reset() {
    if (perf_clock::fd_ptr_) {
      ::ioctl(*perf_clock::fd_ptr_, PERF_EVENT_IOC_RESET, 0);
      return true;
    } else {
      return false;
    }
  }
};

/**
 * @brief   A clock source using hardware CPU cycles accessed via Linux'es
 *          performance counters.
 */
class perf_sw_clock {
 public:
  typedef std::chrono::nanoseconds duration;
  typedef duration::rep rep;
  typedef duration::period period;
  typedef std::chrono::time_point<perf_sw_clock> time_point;
  static constexpr bool is_steady = true;
  inline static fd_ptr_t fd_ptr_  = nullptr;

  /**
   * @brief   Reads a value from the performance events file descriptor
   *
   * @return  duration::rep  The software CPU cycles
   */
  static duration::rep raw() noexcept {
    long long value;

    if (perf_sw_clock::fd_ptr_) {
      return ::read(*perf_sw_clock::fd_ptr_, &value, sizeof(value)) <
                     sizeof(value)
                 ? duration::rep{0}
                 : static_cast<duration::rep>(value);
    } else {
      return duration::rep{0};
    }
  }

  static time_point now() noexcept {
    static auto init  = perf_sw_clock::init();
    static auto reset = perf_sw_clock::reset();

    auto _ = perf_sw_clock::raw();
    return time_point{duration{_}};
  }

  /**
   * @brief Initialises the access to performance counters.
   */
  static bool init() {
    static struct ::perf_event_attr _;
    _.type                     = ::PERF_TYPE_SOFTWARE;
    _.config                   = ::PERF_COUNT_SW_CPU_CLOCK;
    _.size                     = sizeof(_);
    _.exclude_kernel           = 1;
    _.exclude_hv               = 1;
    _.exclude_callchain_kernel = 1;

    if (perf_sw_clock::fd_ptr_) perf_sw_clock::fd_ptr_.reset(nullptr);
    perf_sw_clock::fd_ptr_.reset(
        new long{::syscall(__NR_perf_event_open, &_, 0, -1, -1, 0)});

    return (*perf_sw_clock::fd_ptr_ >= 0);
  }

  /**
   * @brief Resets access to performance counters.
   */
  static bool reset() {
    if (perf_sw_clock::fd_ptr_) {
      ::ioctl(*perf_sw_clock::fd_ptr_, PERF_EVENT_IOC_RESET, 0);
      ::ioctl(*perf_sw_clock::fd_ptr_, PERF_EVENT_IOC_ENABLE, 0);
      return true;
    } else {
      return false;
    }
  }
};

}  // namespace exot::primitives
