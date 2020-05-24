/**
 * @file primitives/tsc.h
 * @author     Bruno Klopott
 * @brief      Wrapper for accessing the time stamp counter.
 */

#pragma once

#include <cstdint>      // for uint*
#include <type_traits>  // for type traits

namespace exot::primitives {

/**
 * @brief      Base class for timestamp counters
 */
struct TimeStampCounter {};

/**
 * @brief      Is the provided type a time stamp counter?
 *
 * @tparam     T          A type to check
 * @tparam     <unnamed>  Template helper
 */
template <typename T, typename = void>
struct is_time_stamp_counter : public std::false_type {};

template <typename T>
struct is_time_stamp_counter<T,
                             std::void_t<decltype(T::start), decltype(T::stop)>>
    : public std::true_type {};

template <typename T>
inline constexpr bool is_time_stamp_counter_v = is_time_stamp_counter<T>::value;

#ifdef __x86_64__

inline namespace x86_64 {

/**
 * @brief      Wrapper for time-measurement functions accessing the time stamp
 *             counter
 * @details    The functions use the improved benchmarking method outlined in
 *             Paoloni G., "How to Benchmark Code Execution Times on Intel®
 *             IA-32 and IA-64 Instruction Set Architectures", Intel
 *             Corporation, September 2010.
 *
 *             The difficulty in using the time stamp counter for profiling and
 *             doing high precision time measurement arises from the fact that
 *             in out-of-order execution these functions are not guaranteed to
 *             execute before/after the code to be profiled/measured.
 *
 *             With the instructions `cpuid` and `rdtscp` we obtain guarantees
 *             that all previous instructions have been executed before the time
 *             stamp counter is read.
 */
struct SerialisingTSC : TimeStampCounter {
  /**
   * @brief      Performs the first read operation from time stamp counter
   *
   * @return     The value of the time stamp counter
   */
  static inline __attribute__((always_inline)) std::uint64_t start() {
    std::uint32_t low, high;

    asm volatile(
        "cpuid           # EXOT->tsc \n\t"  //
        "rdtsc           # EXOT->tsc \n\t"  //
        "movl %%eax, %0  # EXOT->tsc \n\t"  //
        "movl %%edx, %1  # EXOT->tsc     "  //
        : "=r"(low), "=r"(high)             // output
        :                                   // input
        : "%rax", "%rbx", "%rcx", "%rdx");  // clobber

    return (std::uint64_t)high << 32 | (std::uint64_t)low;
  }

  /**
   * @brief      Performs the subsequent read operation from the time stamp
   *             counter
   *
   * @return     The value of the time stamp counter
   */
  static inline __attribute__((always_inline)) std::uint64_t stop() {
    std::uint32_t low, high;

    asm volatile(
        "rdtscp          # EXOT->tsc \n\t"  //
        "movl %%eax, %0  # EXOT->tsc \n\t"  //
        "movl %%edx, %1  # EXOT->tsc \n\t"  //
        "cpuid           # EXOT->tsc     "  //
        : "=r"(low), "=r"(high)             // output
        :                                   // input
        : "%rax", "%rbx", "%rcx", "%rdx");  // clobber

    return (std::uint64_t)high << 32 | (std::uint64_t)low;
  }
};

/**
 * @brief      Simple TSC without cpuid serialisation
 */
struct SimpleTSC : TimeStampCounter {
  static inline __attribute__((always_inline)) std::uint64_t start() {
    std::uint32_t low, high;

    asm volatile(
        "rdtsc           # EXOT->tsc \n\t"  //
        "movl %%eax, %0  # EXOT->tsc \n\t"  //
        "movl %%edx, %1  # EXOT->tsc     "  //
        : "=r"(low), "=r"(high)             // output
        :                                   // input
        : "%rax", "%rbx", "%rcx", "%rdx");  // clobber

    return (std::uint64_t)high << 32 | (std::uint64_t)low;
  }

  static inline __attribute__((always_inline)) std::uint64_t stop() {
    std::uint32_t low, high;

    asm volatile(
        "rdtscp          # EXOT->tsc \n\t"  //
        "movl %%eax, %0  # EXOT->tsc \n\t"  //
        "movl %%edx, %1  # EXOT->tsc     "  //
        : "=r"(low), "=r"(high)             // output
        :                                   // input
        : "%rax", "%rbx", "%rcx", "%rdx");  // clobber

    return static_cast<std::uint64_t>(high) << 32 |
           static_cast<std::uint64_t>(low);
  }
};

/**
 * @brief      Serialising TSC wrapped in load fences
 */
struct LoadFencedTSC : TimeStampCounter {
  static inline __attribute__((always_inline)) std::uint64_t start() {
    std::uint32_t low, high;

    asm volatile(
        "lfence          # EXOT->tsc \n\t"  //
        "cpuid           # EXOT->tsc \n\t"  //
        "rdtsc           # EXOT->tsc \n\t"  //
        "movl %%eax, %0  # EXOT->tsc \n\t"  //
        "movl %%edx, %1  # EXOT->tsc     "  //
        : "=r"(low), "=r"(high)             // output
        :                                   // input
        : "%rax", "%rbx", "%rcx", "%rdx");  // clobber

    return static_cast<std::uint64_t>(high) << 32 |
           static_cast<std::uint64_t>(low);
  }

  static inline __attribute__((always_inline)) std::uint64_t stop() {
    std::uint32_t low, high;

    asm volatile(
        "rdtscp          # EXOT->tsc \n\t"  //
        "movl %%eax, %0  # EXOT->tsc \n\t"  //
        "movl %%edx, %1  # EXOT->tsc \n\t"  //
        "cpuid           # EXOT->tsc \n\t"  //
        "lfence          # EXOT->tsc     "  //
        : "=r"(low), "=r"(high)             // output
        :                                   // input
        : "%rax", "%rbx", "%rcx", "%rdx");  // clobber

    return static_cast<std::uint64_t>(high) << 32 |
           static_cast<std::uint64_t>(low);
  }
};

/**
 * @brief      Serialising TSC wrapped in memory fences
 */
struct MemoryFencedTSC : TimeStampCounter {
  static inline __attribute__((always_inline)) std::uint64_t start() {
    std::uint32_t low, high;

    asm volatile(
        "mfence          # EXOT->tsc \n\t"  //
        "cpuid           # EXOT->tsc \n\t"  //
        "rdtsc           # EXOT->tsc \n\t"  //
        "movl %%eax, %0  # EXOT->tsc \n\t"  //
        "movl %%edx, %1  # EXOT->tsc     "  //
        : "=r"(low), "=r"(high)             // output
        :                                   // input
        : "%rax", "%rbx", "%rcx", "%rdx");  // clobber

    return static_cast<std::uint64_t>(high) << 32 |
           static_cast<std::uint64_t>(low);
  }

  static inline __attribute__((always_inline)) std::uint64_t stop() {
    std::uint32_t low, high;

    asm volatile(
        "rdtscp          # EXOT->tsc \n\t"  //
        "movl %%eax, %0  # EXOT->tsc \n\t"  //
        "movl %%edx, %1  # EXOT->tsc \n\t"  //
        "cpuid           # EXOT->tsc \n\t"  //
        "mfence          # EXOT->tsc     "  //
        : "=r"(low), "=r"(high)             // output
        :                                   // input
        : "%rax", "%rbx", "%rcx", "%rdx");  // clobber

    return static_cast<std::uint64_t>(high) << 32 |
           static_cast<std::uint64_t>(low);
  }
};

/**
 * @brief      Serialising TSC used in prefetch exot channel
 */
struct MemoryFencedPrefetchTSC : TimeStampCounter {
  static inline __attribute__((always_inline)) std::uint64_t start() {
    std::uint32_t low, high;

    asm volatile(
        "mfence          # EXOT->tsc \n\t"  //
        "rdtscp          # EXOT->tsc \n\t"  //
        "movl %%eax, %0  # EXOT->tsc \n\t"  //
        "movl %%edx, %1  # EXOT->tsc \n\t"  //
        "cpuid           # EXOT->tsc     "  //
        : "=r"(low), "=r"(high)             // output
        :                                   // input
        : "%rax", "%rbx", "%rcx", "%rdx");  // clobber

    return static_cast<std::uint64_t>(high) << 32 |
           static_cast<std::uint64_t>(low);
  }

  static inline __attribute__((always_inline)) std::uint64_t stop() {
    std::uint32_t low, high;

    asm volatile(
        "cpuid           # EXOT->tsc \n\t"  //
        "rdtscp          # EXOT->tsc \n\t"  //
        "movl %%eax, %0  # EXOT->tsc \n\t"  //
        "movl %%edx, %1  # EXOT->tsc \n\t"  //
        "mfence          # EXOT->tsc     "  //
        : "=r"(low), "=r"(high)             // output
        :                                   // input
        : "%rax", "%rbx", "%rcx", "%rdx");  // clobber

    return static_cast<std::uint64_t>(high) << 32 |
           static_cast<std::uint64_t>(low);
  }
};

/**
 * @brief      Serialising TSC used in flush-flush exot channel
 */
struct MemoryFencedFlushTSC : TimeStampCounter {
  static inline __attribute__((always_inline)) std::uint64_t start() {
    std::uint32_t low, high;

    asm volatile(
        "mfence          # EXOT->tsc \n\t"  //
        "rdtscp          # EXOT->tsc \n\t"  //
        "mfence          # EXOT->tsc \n\t"  //
        "movl %%eax, %0  # EXOT->tsc \n\t"  //
        "movl %%edx, %1  # EXOT->tsc     "  //
        : "=r"(low), "=r"(high)             // output
        :                                   // input
        : "%rax", "%rbx", "%rcx", "%rdx");  // clobber

    return static_cast<std::uint64_t>(high) << 32 |
           static_cast<std::uint64_t>(low);
  }

  static inline __attribute__((always_inline)) std::uint64_t stop() {
    std::uint32_t low, high;

    asm volatile(
        "mfence          # EXOT->tsc \n\t"  //
        "rdtscp          # EXOT->tsc \n\t"  //
        "mfence          # EXOT->tsc \n\t"  //
        "movl %%eax, %0  # EXOT->tsc \n\t"  //
        "movl %%edx, %1  # EXOT->tsc     "  //
        : "=r"(low), "=r"(high)             // output
        :                                   // input
        : "%rax", "%rbx", "%rcx", "%rdx");  // clobber

    return static_cast<std::uint64_t>(high) << 32 |
           static_cast<std::uint64_t>(low);
  }
};

/**
 * @brief      Another serialising TSC used in flush-flush exot channel
 */
struct MemoryFencedSerialisingFlushTSC : TimeStampCounter {
  static inline __attribute__((always_inline)) std::uint64_t start() {
    std::uint32_t low, high;

    asm volatile(
        "mfence          # EXOT->tsc \n\t"  //
        "cpuid           # EXOT->tsc \n\t"  //
        "rdtsc           # EXOT->tsc \n\t"  //
        "movl %%eax, %0  # EXOT->tsc \n\t"  //
        "movl %%edx, %1  # EXOT->tsc \n\t"  //
        "mfence          # EXOT->tsc     "  //
        : "=r"(low), "=r"(high)             // output
        :                                   // input
        : "%rax", "%rbx", "%rcx", "%rdx");  // clobber

    return static_cast<std::uint64_t>(high) << 32 |
           static_cast<std::uint64_t>(low);
  }

  static inline __attribute__((always_inline)) std::uint64_t stop() {
    std::uint32_t low, high;

    asm volatile(
        "mfence          # EXOT->tsc \n\t"  //
        "rdtscp          # EXOT->tsc \n\t"  //
        "movl %%eax, %0  # EXOT->tsc \n\t"  //
        "movl %%edx, %1  # EXOT->tsc \n\t"  //
        "cpuid           # EXOT->tsc \n\t"  //
        "mfence          # EXOT->tsc     "  //
        : "=r"(low), "=r"(high)             // output
        :                                   // input
        : "%rax", "%rbx", "%rcx", "%rdx");  // clobber

    return static_cast<std::uint64_t>(high) << 32 |
           static_cast<std::uint64_t>(low);
  }
};

}  // namespace x86_64

#endif

}  // namespace exot::primitives
