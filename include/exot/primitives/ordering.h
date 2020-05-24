/**
 * @file primitives/ordering.h
 * @author     Bruno Klopott
 * @brief      Platform specific ordering primitives (fences, etc.).
 */

#pragma once

namespace exot::primitives {

#if defined(__x86_64__)

inline __attribute__((always_inline)) void load_fence() {
  asm volatile("lfence" : : : "memory");
}

inline __attribute__((always_inline)) void store_fence() {
  asm volatile("lfence" : : : "memory");
}

inline __attribute__((always_inline)) void memory_fence() {
  asm volatile("mfence" : : : "memory");
}

inline __attribute__((always_inline)) void full_fence() {
  asm volatile(
      "cpuid   \n\t"
      "mfence      "
      :
      :
      : "memory", "%rax", "%rbx", "%rcx", "%rdx");
}

#elif defined(__arm__) || defined(__aarch64__)

inline __attribute__((always_inline)) void load_fence() {
  asm volatile("dmb ishld" : : : "memory");
}

inline __attribute__((always_inline)) void store_fence() {
  asm volatile("dmb ishst" : : : "memory");
}

inline __attribute__((always_inline)) void memory_fence() {
  asm volatile("dmb ish" : : : "memory");
}

inline __attribute__((always_inline)) void full_fence() {
  asm volatile(
      "dsb sy  \n\t"  // full system reads and writes
      "isb"           // instruction synchronisation barrier
      :
      :
      : "memory");
}

#endif

}  // namespace exot::primitives
