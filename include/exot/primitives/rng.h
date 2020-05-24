/**
 * @file primitives/rng.h
 * @author     Bruno Klopott
 * @brief      Utilities to call hardware random number generator.
 */

#pragma once

/**
 * Only supported on x86_64 platforms (Intel in particular)
 */
#if defined(__x86_64__)

#include <cstdint>  // for explicit size ints
#include <utility>  // for std::pair

namespace exot::primitives {

inline namespace x86_64 {

/**
 * @brief      Gets a 16-bit random number and operation status
 *
 * @return     a pair <status, value>
 */
static inline __attribute__((always_inline)) auto rand16()
    -> std::pair<std::uint8_t, std::uint16_t> {
  unsigned char ok;
  std::uint16_t rand;
  /* rdrand (and rdseed) indicate whether the operation was successful by
   * setting the carry flag (CF), therefore 'cc' is added as a clobber argument,
   * since the code modifies the flag registers. */
  asm volatile("rdrand %0; setc %1" : "=r"(rand), "=rm"(ok) : : "cc");
  return {ok, rand};
}

/**
 * @brief      Gets a 32-bit random number and operation status
 *
 * @return     a pair <status, value>
 */
static inline __attribute__((always_inline)) auto rand32()
    -> std::pair<std::uint8_t, std::uint32_t> {
  unsigned char ok;
  std::uint32_t rand;
  asm volatile("rdrand %0; setc %1" : "=r"(rand), "=rm"(ok) : : "cc");
  return {ok, rand};
}

/**
 * @brief      Gets a 64-bit random number and operation status
 *
 * @return     a pair <status, value>
 */
static inline __attribute__((always_inline)) auto rand64()
    -> std::pair<std::uint8_t, std::uint64_t> {
  unsigned char ok;
  std::uint64_t rand;
  asm volatile("rdrand %0; setc %1" : "=r"(rand), "=rm"(ok) : : "cc");
  return {ok, rand};
}

/**
 * @brief      Gets a 16-bit random number and operation status
 *
 * @return     a pair <status, value>
 */
static inline __attribute__((always_inline)) auto seed16()
    -> std::pair<std::uint8_t, std::uint16_t> {
  unsigned char ok;
  std::uint16_t rand;
  asm volatile("rdseed %0; setc %1" : "=r"(rand), "=rm"(ok) : : "cc");
  return {ok, rand};
}

/**
 * @brief      Gets a 32-bit random number and operation status
 *
 * @return     a pair <status, value>
 */
static inline __attribute__((always_inline)) auto seed32()
    -> std::pair<std::uint8_t, std::uint32_t> {
  unsigned char ok;
  std::uint32_t rand;
  asm volatile("rdseed %0; setc %1" : "=r"(rand), "=rm"(ok) : : "cc");
  return {ok, rand};
}

/**
 * @brief      Gets a 64-bit random number and operation status
 *
 * @return     a pair <status, value>
 */
static inline __attribute__((always_inline)) auto seed64()
    -> std::pair<std::uint8_t, std::uint64_t> {
  unsigned char ok;
  std::uint64_t rand;
  asm volatile("rdseed %0; setc %1" : "=r"(rand), "=rm"(ok) : : "cc");
  return {ok, rand};
}

/**
 * @brief      Tries to get a 16-bit random number until successful
 *
 * @return     The random number
 */
static inline __attribute__((always_inline)) auto rand16_until_ok()
    -> std::uint16_t {
  std::uint16_t rand;
  asm volatile(
      "1:         \n\t"
      "rdrand %0  \n\t"
      "jnc 1b     \n\t"
      : "=r"(rand)
      :
      : "cc");
  return rand;
}

/**
 * @brief      Tries to get a 32-bit random number until successful
 *
 * @return     The random number
 */
static inline __attribute__((always_inline)) auto rand32_until_ok()
    -> std::uint32_t {
  std::uint32_t rand;
  asm volatile(
      "1:         \n\t"
      "rdrand %0  \n\t"
      "jnc 1b     \n\t"
      : "=r"(rand)
      :
      : "cc");
  return rand;
}

/**
 * @brief      Tries to get a 64-bit random number until successful
 *
 * @return     The random number
 */
static inline __attribute__((always_inline)) auto rand64_until_ok()
    -> std::uint64_t {
  std::uint64_t rand;
  asm volatile(
      "1:         \n\t"
      "rdrand %0  \n\t"
      "jnc 1b     \n\t"
      : "=r"(rand)
      :
      : "cc");
  return rand;
}

/**
 * @brief      Tries to get a 16-bit random number until successful
 *
 * @return     The random number
 */
static inline __attribute__((always_inline)) auto seed16_until_ok()
    -> std::uint16_t {
  std::uint16_t rand;
  asm volatile(
      "1:         \n\t"
      "rdseed %0  \n\t"
      "jnc 1b     \n\t"
      : "=r"(rand)
      :
      : "cc");
  return rand;
}

/**
 * @brief      Tries to get a 32-bit random number until successful
 *
 * @return     The random number
 */
static inline __attribute__((always_inline)) auto seed32_until_ok()
    -> std::uint32_t {
  std::uint32_t rand;
  asm volatile(
      "1:         \n\t"
      "rdseed %0  \n\t"
      "jnc 1b     \n\t"
      : "=r"(rand)
      :
      : "cc");
  return rand;
}

/**
 * @brief      Tries to get a 64-bit random number until successful
 *
 * @return     The random number
 */
static inline __attribute__((always_inline)) auto seed64_until_ok()
    -> std::uint64_t {
  std::uint64_t rand;
  asm volatile(
      "1:         \n\t"
      "rdseed %0  \n\t"
      "jnc 1b     \n\t"
      : "=r"(rand)
      :
      : "cc");
  return rand;
}

/**
 * @brief      Check if a 16-bit random number can be obtained
 *
 * @param      ok    The status
 */
static inline __attribute__((always_inline)) void check_rand16(
    std::uint8_t& ok) {
  std::uint16_t rand;
  asm volatile("rdrand %0; setc %1" : "=r"(rand), "=rm"(ok) : : "cc");
}

/**
 * @brief      Check if a 32-bit random number can be obtained
 *
 * @param      ok    The status
 */
static inline __attribute__((always_inline)) void check_rand32(
    std::uint8_t& ok) {
  std::uint32_t rand;
  asm volatile("rdrand %0; setc %1" : "=r"(rand), "=rm"(ok) : : "cc");
}

/**
 * @brief      Check if a 64-bit random number can be obtained
 *
 * @param      ok    The status
 */
static inline __attribute__((always_inline)) void check_rand64(
    std::uint8_t& ok) {
  std::uint64_t rand;
  asm volatile("rdrand %0; setc %1" : "=r"(rand), "=rm"(ok) : : "cc");
}

/**
 * @brief      Check if a 16-bit random number can be obtained
 *
 * @param      ok    The status
 */
static inline __attribute__((always_inline)) void check_seed16(
    std::uint8_t& ok) {
  std::uint16_t rand;
  asm volatile("rdseed %0; setc %1" : "=r"(rand), "=rm"(ok) : : "cc");
}

/**
 * @brief      Check if a 32-bit random number can be obtained
 *
 * @param      ok    The status
 */
static inline __attribute__((always_inline)) void check_seed32(
    std::uint8_t& ok) {
  std::uint32_t rand;
  asm volatile("rdseed %0; setc %1" : "=r"(rand), "=rm"(ok) : : "cc");
}

/**
 * @brief      Check if a 64-bit random number can be obtained
 *
 * @param      ok    The status
 */
static inline __attribute__((always_inline)) void check_seed64(
    std::uint8_t& ok) {
  std::uint64_t rand;
  asm volatile("rdseed %0; setc %1" : "=r"(rand), "=rm"(ok) : : "cc");
}

}  // namespace x86_64

}  // namespace exot::primitives

#endif
