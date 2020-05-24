/**
 * @file utilities/alignment.h
 * @author     Bruno Klopott
 * @brief      Implementation of an evicter using an eviction strategy.
 */

#pragma once

#include <type_traits>

#if defined(Evicter_DEBUG)
#include <fmt/format.h>
#define EVICT_DBG_PRINT(txt, a1, i, j, k, a2, v)                               \
  do {                                                                         \
    fmt::print(                                                                \
        stderr,                                                                \
        "{:>20s} (@ {:16} [  i,  j,  k] = [{:3},{:3},{:3}] <> @ {:16} = {}\n", \
        txt, fmt::ptr(a1), i, j, k, fmt::ptr(a2), v);                          \
  } while (false)
#else
#define EVICT_DBG_PRINT(txt, a1, i, j, k, a2, v)
#endif

#include <exot/utilities/types.h>

namespace exot::utilities {

/**
 * @brief      Accesses memory with specific patterns.
 */
struct Evicter {
 private:
  std::size_t addrs_;
  std::size_t accss_;
  std::size_t ovrlp_;

  std::size_t fwd_i_lim;
  std::size_t fwd_k_lim;
  std::size_t rev_i_lim;
  std::size_t rev_k_lim;

 public:
  /**
   * @brief      Constructs the evicter
   *
   * @param[in]  addresses_per_loop  The number of addresses per loop
   * @param[in]  accesses_per_loop   The number of accesses to the same address
   *                                 per loop
   * @param[in]  overlap_parameter   The overlap parameter (step size)
   */
  explicit Evicter(std::size_t addresses_per_loop,  //
                   std::size_t accesses_per_loop,   //
                   std::size_t overlap_parameter)
      : addrs_{addresses_per_loop},
        accss_{accesses_per_loop},
        ovrlp_{overlap_parameter} {}

  /**
   * @brief      Constructs the evicter
   *
   * @param[in]  addresses_per_loop  The number of addresses per loop
   * @param[in]  accesses_per_loop   The number of accesses to the same address
   *                                 per loop
   */
  explicit Evicter(std::size_t addresses_per_loop,
                   std::size_t accesses_per_loop)
      : Evicter(addresses_per_loop, accesses_per_loop, 1) {}

  /**
   * @brief      Default constructor
   */
  Evicter() : Evicter(1u, 1u) {}

  /**
   * @brief      Accesses (reads) addresses using a buffer
   *
   * @param      buffer     The buffer
   *
   * @tparam     T          The type of the buffer
   * @tparam     <unnamed>  Template helper (only types with methods `size`,
   *                        `at, `operator[]`, and with non-pointer value type)
   */
  template <typename T,
            typename = std::enable_if_t<(
                exot::utilities::is_location_accessible<T>::value &&
                !std::is_pointer<typename std::decay_t<T>::value_type>::value)>>
  void read(T& buffer) {
    static auto size = buffer.size();
    if (addrs_ >= size) throw std::out_of_range("addrs_ >= size");

    auto i_end = size - addrs_;
    for (auto i = std::size_t{0}; i <= i_end; i += ovrlp_) {  // [0, i_end]
      for (auto j = std::size_t{0}; j < accss_; ++j) {        // [0, accss_)
        for (auto k = std::size_t{0}; k < addrs_; ++k) {      // [0, addrs_)
          volatile auto _ = buffer[i + k];
          EVICT_DBG_PRINT("read", std::addressof(buffer), i, j, k,
                          std::addressof(buffer[i + k]), _);
        }
      }
    }
  }

  /**
   * @brief      Accesses (reads) addresses contained in a buffer
   *
   * @param      buffer     The buffer
   *
   * @tparam     T          The type of the buffer
   * @tparam     <unnamed>  Template helper (only types with methods `size`,
   *                        `at, `operator[]`, and with pointer value type)
   */
  template <typename T,
            typename = std::enable_if_t<
                (exot::utilities::is_location_accessible<T>::value &&
                 std::is_pointer<typename std::decay_t<T>::value_type>::value)>>
  void read_ptrs(T& buffer) {
    static auto size = buffer.size();
    if (addrs_ >= size) throw std::out_of_range("addrs_ >= size");

    auto i_end = size - addrs_;
    for (auto i = std::size_t{0}; i <= i_end; i += ovrlp_) {
      for (auto j = std::size_t{0}; j < accss_; ++j) {
        for (auto k = std::size_t{0}; k < addrs_; ++k) {
          volatile auto _ = *(buffer[i + k]);
          EVICT_DBG_PRINT("read_ptrs", std::addressof(buffer), i, j, k,
                          buffer[i + k], _);
        }
      }
    }
  }

  /**
   * @brief      Accesses (reads) addresses using a base pointer and length
   *
   * @param      address    The address
   * @param[in]  length     The length
   *
   * @tparam     T          The type
   * @tparam     <unnamed>  Template helper (only pointer types)
   */
  template <typename T, typename = std::enable_if_t<std::is_pointer<T>::value>>
  void read(T address, std::size_t length) {
    if (addrs_ >= length) throw std::out_of_range("addrs_ >= length");

    auto i_end = length - addrs_;
    for (auto i = std::size_t{0}; i <= i_end; i += ovrlp_) {
      for (auto j = std::size_t{0}; j < accss_; ++j) {
        for (auto k = std::size_t{0}; k < addrs_; ++k) {
          volatile auto _ = *(address + i + k);
          EVICT_DBG_PRINT("read", address, i, j, k, (address + i + k), _);
        }
      }
    }
  }

  /**
   * @brief      Accesses (writes) addresses using a buffer
   *
   * @param      buffer     The buffer
   *
   * @tparam     T          The type of the buffer
   * @tparam     <unnamed>  Template helper (only types with methods `size`,
   *                        `at, `operator[]`, and with non-pointer value type)
   */
  template <typename T,
            typename = std::enable_if_t<(
                exot::utilities::is_location_accessible<T>::value &&
                !std::is_pointer<typename std::decay_t<T>::value_type>::value)>>
  void write(T& buffer, volatile typename T::value_type value) {
    static auto size = buffer.size();
    if (addrs_ >= size) throw std::out_of_range("addrs_ >= size");

    auto i_end = size - addrs_;
    for (auto i = std::size_t{0}; i <= i_end; i += ovrlp_) {
      for (auto j = std::size_t{0}; j < accss_; ++j) {
        for (auto k = std::size_t{0}; k < addrs_; ++k) {
          buffer[i + k] = value;
          EVICT_DBG_PRINT("write", std::addressof(buffer), i, j, k,
                          std::addressof(buffer[i + k]), value);
        }
      }
    }
  }

  /**
   * @brief      Accesses (writes) addresses contained in a buffer
   *
   * @param      buffer     The buffer
   * @param[in]  value      The value
   *
   * @tparam     T          The type of the buffer
   * @tparam     <unnamed>  Template helper (only types with methods `size`,
   *                        `at, `operator[]`, and with pointer value type)
   */
  template <typename T,
            typename = std::enable_if_t<
                (exot::utilities::is_location_accessible<T>::value &&
                 std::is_pointer<typename std::decay_t<T>::value_type>::value)>>
  void write_ptrs(T& buffer, volatile typename T::value_type value) {
    static auto size = buffer.size();
    if (addrs_ >= size) throw std::out_of_range("addrs_ >= size");

    auto i_end = size - addrs_;
    for (auto i = std::size_t{0}; i <= i_end; i += ovrlp_) {
      for (auto j = std::size_t{0}; j < accss_; ++j) {
        for (auto k = std::size_t{0}; k < addrs_; ++k) {
          *(buffer[i + k]) = value;
          EVICT_DBG_PRINT("write_ptrs", std::addressof(buffer), i, j, k,
                          buffer[i + k], value);
        }
      }
    }
  }

  /**
   * @brief      Accesses (writes) addresses using a base pointer and length
   *
   * @param      address    The address
   * @param[in]  length     The length
   *
   * @tparam     T          The type
   * @tparam     <unnamed>  Template helper (only pointer types)
   */
  template <typename T, typename = std::enable_if_t<std::is_pointer<T>::value>>
  void write(T address, std::size_t length, volatile T value) {
    if (addrs_ >= length) throw std::out_of_range("addrs_ >= length");

    auto i_end = length - addrs_;
    for (auto i = std::size_t{0}; i <= i_end; i += ovrlp_) {
      for (auto j = std::size_t{0}; j < accss_; ++j) {
        for (auto k = std::size_t{0}; k < addrs_; ++k) {
          *(address + i + k) = value;
          EVICT_DBG_PRINT("write", address, i, j, k, (address + i + k), value);
        }
      }
    }
  }

  /****************************************************************************
   *                                 REVERSED                                 *
   ****************************************************************************/

  /**
   * @brief      Accesses (reads) addresses using a buffer in reverse order
   *
   * @param      buffer     The buffer
   *
   * @tparam     T          The type of the buffer
   * @tparam     <unnamed>  Template helper (only types with methods `size`,
   *                        `at, `operator[]`, and with non-pointer value type)
   */
  template <typename T,
            typename = std::enable_if_t<(
                exot::utilities::is_location_accessible<T>::value &&
                !std::is_pointer<typename std::decay_t<T>::value_type>::value)>>
  void read_in_reverse(T& buffer) {
    using ssize_t = signed long long;

    static auto size = buffer.size();
    if (addrs_ >= size) throw std::out_of_range("addrs_ >= size");

    auto i_begin = static_cast<ssize_t>(size - addrs_);
    auto k_begin = static_cast<ssize_t>(addrs_) - 1;

    for (auto i = i_begin; i >= ssize_t{0}; i -= ovrlp_) {  // [i_begin, 0]
      for (auto j = std::size_t{0}; j < accss_; ++j) {      // [0, accss_)
        for (auto k = k_begin; k >= ssize_t{0}; --k) {      // [k_begin, 0]
          volatile auto _ = buffer[i + k];
          EVICT_DBG_PRINT("read_in_reverse", std::addressof(buffer), i, j, k,
                          std::addressof(buffer[i + k]), _);
        }
      }
    }
  }

  /**
   * @brief      Accesses (reads) addresses contained in a buffer in reverse
   *             order
   *
   * @param      buffer     The buffer
   *
   * @tparam     T          The type of the buffer
   * @tparam     <unnamed>  Template helper (only types with methods `size`,
   *                        `at, `operator[]`, and with pointer value type)
   */
  template <typename T,
            // typename = std::void_t<typename std::decay_t<T>::value_type>,
            typename = std::enable_if_t<
                (exot::utilities::is_location_accessible<T>::value &&
                 std::is_pointer<typename std::decay_t<T>::value_type>::value)>>
  void read_ptrs_in_reverse(T& buffer) {
    using ssize_t = signed long long;

    static auto size = buffer.size();
    if (addrs_ >= size) throw std::out_of_range("addrs_ >= size");

    auto i_begin = static_cast<ssize_t>(size - addrs_);
    auto k_begin = static_cast<ssize_t>(addrs_) - 1;

    for (auto i = i_begin; i >= ssize_t{0}; i -= ovrlp_) {
      for (auto j = std::size_t{0}; j < accss_; ++j) {
        for (auto k = k_begin; k >= ssize_t{0}; --k) {
          volatile auto _ = *(buffer[i + k]);
          EVICT_DBG_PRINT("read_ptrs_in_reverse", std::addressof(buffer), i, j,
                          k, buffer[i + k], _);
        }
      }
    }
  }

  /**
   * @brief      Accesses (reads) addresses using a base pointer and length in
   *             reverse order
   *
   * @param      address    The address
   * @param[in]  length     The length
   *
   * @tparam     T          The type
   * @tparam     <unnamed>  Template helper (only pointer types)
   */
  template <typename T, typename = std::enable_if_t<std::is_pointer<T>::value>>
  void read_in_reverse(T address, std::size_t length) {
    using ssize_t = signed long long;
    if (addrs_ >= length) throw std::out_of_range("addrs_ >= length");

    auto i_begin = static_cast<ssize_t>(length - addrs_);
    auto k_begin = static_cast<ssize_t>(addrs_) - 1;

    for (auto i = i_begin; i >= ssize_t{0}; i -= ovrlp_) {
      for (auto j = std::size_t{0}; j < accss_; ++j) {
        for (auto k = k_begin; k >= ssize_t{0}; --k) {
          volatile auto _ = *(address + i + k);
          EVICT_DBG_PRINT("read_in_reverse", address, i, j, k,
                          (address + i + k), _);
        }
      }
    }
  }

  /**
   * @brief      Accesses (writes) addresses using a buffer in reverse order
   *
   * @param      buffer     The buffer
   * @param[in]  value      The value
   *
   * @tparam     T          The type of the buffer
   * @tparam     <unnamed>  Template helper (only types with methods `size`,
   *                        `at, `operator[]`, and with non-pointer value type)
   */
  template <typename T,
            typename = std::enable_if_t<(
                exot::utilities::is_location_accessible<T>::value &&
                !std::is_pointer<typename std::decay_t<T>::value_type>::value)>>
  void write_in_reverse(T& buffer, volatile typename T::value_type value) {
    using ssize_t = signed long long;

    static auto size = buffer.size();
    if (addrs_ >= size) throw std::out_of_range("addrs_ >= size");

    auto i_begin = static_cast<ssize_t>(size - addrs_);
    auto k_begin = static_cast<ssize_t>(addrs_) - 1;

    for (auto i = i_begin; i >= ssize_t{0}; i -= ovrlp_) {
      for (auto j = std::size_t{0}; j < accss_; ++j) {
        for (auto k = k_begin; k >= ssize_t{0}; --k) {
          buffer[i + k] = value;
          EVICT_DBG_PRINT("write_in_reverse", std::addressof(buffer), i, j, k,
                          std::addressof(buffer[i + k]), value);
        }
      }
    }
  }

  /**
   * @brief      Accesses (writes) addresses contained in a buffer in reverse
   *             order
   *
   * @param      buffer     The buffer
   * @param[in]  value      The value
   *
   * @tparam     T          The type of the buffer
   * @tparam     <unnamed>  Template helper (only types with methods `size`,
   *                        `at, `operator[]`, and with pointer value type)
   */
  template <typename T,
            typename = std::enable_if_t<
                (exot::utilities::is_location_accessible<T>::value &&
                 std::is_pointer<typename std::decay_t<T>::value_type>::value)>>
  void write_ptrs_in_reverse(T& buffer, volatile typename T::value_type value) {
    using ssize_t = signed long long;

    static auto size = buffer.size();
    if (addrs_ >= size) throw std::out_of_range("addrs_ >= size");

    auto i_begin = static_cast<ssize_t>(size - addrs_);
    auto k_begin = static_cast<ssize_t>(addrs_) - 1;

    for (auto i = i_begin; i >= ssize_t{0}; i -= ovrlp_) {
      for (auto j = std::size_t{0}; j < accss_; ++j) {
        for (auto k = k_begin; k >= ssize_t{0}; --k) {
          *(buffer[i + k]) = value;
          EVICT_DBG_PRINT("write_ptrs_in_reverse", std::addressof(buffer), i, j,
                          k, buffer[i + k], value);
        }
      }
    }
  }

  /**
   * @brief      Accesses (writes) addresses using a base pointer and length in
   *             reverse order
   *
   * @param      address    The address
   * @param[in]  length     The length
   * @param[in]  value      The value
   *
   * @tparam     T          The type
   * @tparam     <unnamed>  Template helper (only pointer types)
   */
  template <typename T, typename = std::enable_if_t<std::is_pointer<T>::value>>
  void write_in_reverse(T address, std::size_t length, volatile T value) {
    using ssize_t = signed long long;
    if (addrs_ >= length) throw std::out_of_range("addrs_ >= length");

    auto i_begin = static_cast<ssize_t>(length - addrs_);
    auto k_begin = static_cast<ssize_t>(addrs_) - 1;

    for (auto i = i_begin; i >= ssize_t{0}; i -= ovrlp_) {
      for (auto j = std::size_t{0}; j < accss_; ++j) {
        for (auto k = k_begin; k >= ssize_t{0}; --k) {
          *(address + i + k) = value;
          EVICT_DBG_PRINT("write_in_reverse", address, i, j, k,
                          (address + i + k), value);
        }
      }
    }
  }
};

}  // namespace exot::utilities
