/**
 * @file utilities/thread.cpp
 * @author     Bruno Klopott
 * @brief      Implementation of the thread traits from @ref thread.h.
 */

#include <exot/utilities/thread.h>

#if defined(__linux__)
#include <sys/syscall.h>  // for SYS_gettid
#include <sys/types.h>    // for pid_t
#include <unistd.h>       // for syscall

#elif defined(__APPLE__)
#include <thread>  // for this_thread::get_id

#include <fmt/ostream.h>

#include <exot/utilities/x86_64.h>  // for logical_id
#endif

#include <algorithm>  // for std::clamp

#include <fmt/format.h>  // for fmt::format

#include <exot/utilities/types.h>  // for to_underlying_type

namespace exot::utilities {

std::string thread_info() {
  int policy;
  struct sched_param param;

  pthread_getschedparam(pthread_self(), &policy, &param);

#if !defined(__APPLE__)
  return fmt::format("id: {}, cpu: {}, policy: {}, priority: {}",
                     static_cast<unsigned long>(pthread_self()), sched_getcpu(),
                     policy, param.sched_priority);
#else
  return fmt::format("id: {}, cpu: {}, policy: {}, priority: {}",
                     std::this_thread::get_id(), exot::utilities::logical_id(),
                     policy, param.sched_priority);
#endif
};

int ThreadTraits::set_affinity(unsigned int cpu) {
#if !defined(__APPLE__)
  /* CPU_* macros manipulate the bitmask cpu_mask, are available only on
   * Linux-based platforms. */
  cpu_set_t cpu_mask;
  CPU_ZERO(&cpu_mask);
  CPU_SET(cpu, &cpu_mask);

  /* The syscall `gettid()` is used in conjunction with sched_setaffinity as it
   * seems to be more portable than pthread_setaffinity_np.
   */
  pid_t tid = syscall(SYS_gettid);
  return sched_setaffinity(tid, sizeof(cpu_set_t), &cpu_mask);
#else
  return -1;
#endif
};

int ThreadTraits::set_affinity(pthread_t native_handle, unsigned int cpu) {
#if !defined(__ANDROID__) && !defined(__APPLE__)
  cpu_set_t cpu_mask;
  CPU_ZERO(&cpu_mask);
  CPU_SET(cpu, &cpu_mask);

  return pthread_setaffinity_np(native_handle, sizeof(cpu_set_t), &cpu_mask);
#else
  return -1;
#endif
};

/**
 * @details    The chosen priority will be clamped to the allowed range for a
 *             specific scheduling policy (e.g. [0] only for default policy,
 *             [0,99] for FIFO and round robin). Unexpected behaviour can happen
 *             when the implementation defines lower granularity of priorities,
 *             since in POSIX only a certain minimum number of steps has to be
 *             available.
 */
int ThreadTraits::set_scheduling(SchedulingPolicy policy, int priority) {
#if !defined(__APPLE__)
  /* "Cast" the scoped enum to its underlying type `int`. */
  auto policy_ = to_underlying_type(policy);

  /* Get minimum and maximum priority values for given policy. */
  auto min = sched_get_priority_min(policy_);
  auto max = sched_get_priority_max(policy_);

  /* Clamp given priority to the allowed value range. */
  int priority_ = std::clamp(priority, min, max);

  const struct sched_param param = {priority_};

  pid_t tid = syscall(SYS_gettid);
  return sched_setscheduler(tid, policy_, &param);
#else
  return ThreadTraits::set_scheduling(pthread_self(), policy, priority);
#endif
};

int ThreadTraits::set_scheduling(pthread_t native_handle,
                                 SchedulingPolicy policy, int priority) {
  /* "Cast" the scoped enum to its underlying type `int`. */
  auto policy_ = to_underlying_type(policy);

  /* Get minimum and maximum priority values for given policy. */
  auto min = sched_get_priority_min(policy_);
  auto max = sched_get_priority_max(policy_);

  /* Clamp given priority to the allowed value range. */
  int priority_ = std::clamp(priority, min, max);

  const struct sched_param param = {priority_};

  return pthread_setschedparam(native_handle, policy_, &param);
};

}  // namespace exot::utilities
