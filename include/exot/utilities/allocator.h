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
 * @file utilities/alignment.h
 * @author     Bruno Klopott
 * @brief      Custom allocators complying with STL requirements, for aligned
 *             storage, storage with advices to the kernel, huge pages storage,
 *             and aligned & advised storage.
 */

#pragma once

#if defined(__linux__)
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>
#endif

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <limits>
#include <string>
#include <type_traits>

#include <fmt/format.h>

#include <exot/utilities/alignment.h>
#include <exot/utilities/enum.h>

namespace exot::utilities {

struct bad_alloc : public std::exception {
  bad_alloc(std::string err) : what_{err} {}
  bad_alloc(const char* err) : what_{err} {}
  bad_alloc() : what_{""} {}
  const char* what() const noexcept override { return what_.c_str(); }

 private:
  std::string what_;
};

/**
 * @brief      Enumeration for madvice(2) advice bits.
 * @note       Prevents invalid use and establishes strong types.
 */
enum class Advice : int {
  normal      = MADV_NORMAL,
  random      = MADV_RANDOM,
  sequential  = MADV_SEQUENTIAL,
  willneed    = MADV_WILLNEED,
  dontneed    = MADV_DONTNEED,
  remove      = MADV_REMOVE,
  dontfork    = MADV_DONTFORK,
  dofork      = MADV_DOFORK,
  hwpoison    = MADV_HWPOISON,
  mergeable   = MADV_MERGEABLE,
  unmergeable = MADV_UNMERGEABLE,
  hugepage    = MADV_HUGEPAGE,
  nohugepage  = MADV_NOHUGEPAGE,
  dontdump    = MADV_DONTDUMP,
  dodump      = MADV_DODUMP,
  free        = MADV_FREE,
  keeponfork  = MADV_KEEPONFORK
};

/**
 * @brief      Overload of enum_operations_enabled for Advice
 * @details    Enables bitwise operators on the Advice enum.
 */
template <>
struct enum_operations_enabled<Advice> : std::true_type {};

/**
 * @brief      Advises the kernel about how a memory range is used
 *
 * @param      address  The address
 * @param[in]  length   The length
 * @param[in]  advice   The advice
 */
inline void advise(void* address, std::size_t length, Advice advice) {
  if (::madvise(address, length, static_cast<int>(advice)) != 0)
    throw std::logic_error(
        fmt::format("madvice failed with error: {}",
                    errno != 0 ? std::strerror(errno) : "unknown"));
}

/**
 * @brief      Aligned allocator to use with STL containers
 *
 * @tparam     T     The value type
 * @tparam     Al    The desired alignment
 */
template <typename T, std::size_t Al = alignof(T)>
struct AlignedAllocator {
  /* Required typedefs/aliases */
  using value_type                             = T;
  using size_type                              = std::size_t;
  using difference_type                        = ptrdiff_t;
  using propagate_on_container_move_assignment = std::true_type;
  using is_always_equal                        = std::true_type;

  /* Can only align on alignments that are powers of 2. */
  static_assert(is_valid_alignment(Al), "Alignment must be a power of 2");
  static_assert(alignof(T) <= Al, "Alignment must be at least the default");

  /* Constructors */
  AlignedAllocator()                              = default;
  AlignedAllocator(const AlignedAllocator& other) = default;
  AlignedAllocator& operator=(const AlignedAllocator& other) = delete;
  template <typename U>
  AlignedAllocator(const AlignedAllocator<U, Al>&) noexcept {};

  ~AlignedAllocator() = default;

  /* Deprecated */
  template <class U>
  struct rebind {
    using other = AlignedAllocator<U, Al>;
  };

  /**
   * @brief      Allocates memory
   * @details    Uses the `posix_memalign` function from cstdlib.
   *
   * @param[in]  n     The size to allocate
   */
  [[nodiscard]] T* allocate(std::size_t n) const {
    if (n == 0) { return nullptr; }
    if (n > std::numeric_limits<size_type>::max() / sizeof(T)) {
      throw bad_alloc(
          "AlignedAllocator failed: requested more bytes than possible.");
    }

    /* Allocate memory. posix_memalign returns non-zero if failed. */
    void* aligned = nullptr;
    if (::posix_memalign(&aligned, Al, n * sizeof(T))) {
      throw bad_alloc(fmt::format(
          "AlignedAllocator failed: posix_memalign failed with error: {}.",
          errno != 0 ? std::strerror(errno) : "unknown"));
    }

    if (aligned != nullptr) {
      return reinterpret_cast<T*>(aligned);
    } else {
      throw bad_alloc("AlignedAllocator failed: null pointer");
    }
  }

  /**
   * @brief      Frees memory
   *
   * @param      p     The pointer to memory to deallocate
   * @param[in]  n     The count to deallocate (unused)
   */
  void deallocate(T* p, std::size_t n) const {
    std::free(reinterpret_cast<void*>(p));
  }
};

/**
 * @brief      Checks if allocators are not the same
 *
 * @param[in]  lhs   The left hand side
 * @param[in]  rhs   The right hand side
 *
 * @tparam     T1    The lhs value type
 * @tparam     Al1    The lhs alignment
 * @tparam     T2    The rhs value type
 * @tparam     Al2    The rhs alignment
 *
 * @return     Always false (stateless allocator)
 */
template <typename T1, std::size_t Al1, typename T2, std::size_t Al2>
bool operator!=(const AlignedAllocator<T1, Al1>& lhs,
                const AlignedAllocator<T2, Al2>& rhs) noexcept {
  return false;
}

/**
 * @brief      Checks if allocators are the same
 *
 * @param[in]  lhs   The left hand side
 * @param[in]  rhs   The right hand side
 *
 * @tparam     T1    The lhs value type
 * @tparam     Al1    The lhs alignment
 * @tparam     T2    The rhs value type
 * @tparam     Al2    The rhs alignment
 *
 * @return     Always true (stateless allocator)
 */
template <typename T1, std::size_t Al1, typename T2, std::size_t Al2>
bool operator==(const AlignedAllocator<T1, Al1>& lhs,
                const AlignedAllocator<T2, Al2>& rhs) noexcept {
  return true;
}

#if defined(__linux__)

/**
 * @brief      Allocator using anonymous mmap mappings with allocation advices
 *
 * @tparam     T     The value type
 * @tparam     Ad    Advice flags
 */
template <typename T, Advice Ad = Advice::normal>
struct AdvisedAllocator {
  /* Required typedefs/aliases */
  using value_type                             = T;
  using size_type                              = std::size_t;
  using difference_type                        = ptrdiff_t;
  using propagate_on_container_move_assignment = std::true_type;
  using is_always_equal                        = std::true_type;

  /* Constructors */
  AdvisedAllocator()                              = default;
  AdvisedAllocator(const AdvisedAllocator& other) = default;
  AdvisedAllocator& operator=(const AdvisedAllocator& other) = delete;
  template <typename U>
  AdvisedAllocator(const AdvisedAllocator<U, Ad>&) noexcept {};

  ~AdvisedAllocator() = default;

  /* Deprecated */
  template <class U>
  struct rebind {
    using other = AdvisedAllocator<U, Ad>;
  };

  /**
   * @brief      Allocates memory
   *
   * @param[in]  n     The size to allocate
   */
  [[nodiscard]] T* allocate(std::size_t n) const {
    if (n == 0) { return nullptr; }
    if (n > std::numeric_limits<size_type>::max() / sizeof(T)) {
      throw bad_alloc(
          "AdvisedAllocator failed: requested more bytes than possible.");
    }

    void* mem =
        ::mmap(nullptr, n * sizeof(T), PROT_READ | PROT_WRITE | PROT_EXEC,
               MAP_PRIVATE | MAP_ANONYMOUS, 0, 0);

    if (mem == (void*)(-1))
      throw bad_alloc(
          fmt::format("AdvisedAllocator failed: mmap failed with error: {}.",
                      errno != 0 ? std::strerror(errno) : "unknown"));

    /* Advise the kernel about the use of the mmap'ed memory. */
    if (::madvise(mem, n * sizeof(T), static_cast<int>(Ad)) != 0)
      throw bad_alloc(
          fmt::format("AdvisedAllocator failed: madvise failed with error: {}.",
                      errno != 0 ? std::strerror(errno) : "unknown"));

    if (mem != nullptr) {
      return reinterpret_cast<T*>(mem);
    } else {
      throw bad_alloc("AdvisedAllocator failed: null pointer");
    }
  }

  /**
   * @brief      Frees memory
   *
   * @param      p     The pointer to memory to deallocate
   * @param[in]  n     The size to deallocate
   */
  void deallocate(T* p, std::size_t n) const {
    ::munmap(reinterpret_cast<void*>(p), n);
  }
};

/**
 * @brief      Checks if allocators are not the same
 *
 * @param[in]  lhs   The left hand side
 * @param[in]  rhs   The right hand side
 *
 * @tparam     T1    The lhs value type
 * @tparam     Ad1   The lhs advice
 * @tparam     T2    The rhs value type
 * @tparam     Ad2   The rhs advice
 *
 * @return     Always false (stateless allocator)
 */
template <typename T1, Advice Ad1,  //
          typename T2, Advice Ad2>
bool operator!=(const AdvisedAllocator<T1, Ad1>& lhs,
                const AdvisedAllocator<T2, Ad2>& rhs) noexcept {
  return false;
}

/**
 * @brief      Checks if allocators are the same
 *
 * @param[in]  lhs   The left hand side
 * @param[in]  rhs   The right hand side
 *
 * @tparam     T1    The lhs value type
 * @tparam     Ad1   The lhs advice
 * @tparam     T2    The rhs value type
 * @tparam     Ad2   The rhs advice
 *
 * @return     Always true (stateless allocator)
 */
template <typename T1, Advice Ad1,  //
          typename T2, Advice Ad2>
bool operator==(const AdvisedAllocator<T1, Ad1>& lhs,
                const AdvisedAllocator<T2, Ad2>& rhs) noexcept {
  return true;
}

/**
 * @brief      Allocator using Linux huge pages facilities
 * @details    The use of the Advice is optional, but can be helpful if one
 *             wants to create transparent hugepages (by setting Ad to
 *             Advice::hugepage), or to advise the kernel about page preloading,
 *             etc.
 *
 * @tparam     T     The value type
 * @tparam     Ad     Allocation advice flags
 */
template <typename T, Advice Ad = Advice::normal>
struct HugePagesAllocator {
  /* Required typedefs/aliases */
  using value_type                             = T;
  using size_type                              = std::size_t;
  using difference_type                        = ptrdiff_t;
  using propagate_on_container_move_assignment = std::true_type;
  using is_always_equal                        = std::true_type;

  /* Constructors */
  HugePagesAllocator()                                = default;
  HugePagesAllocator(const HugePagesAllocator& other) = default;
  HugePagesAllocator& operator=(const HugePagesAllocator& other) = delete;
  template <typename U>
  HugePagesAllocator(const HugePagesAllocator<U, Ad>&) noexcept {};

  ~HugePagesAllocator() = default;

  /* Deprecated */
  template <class U>
  struct rebind {
    using other = HugePagesAllocator<U, Ad>;
  };

  /**
   * @brief      Allocates memory
   *
   * @param[in]  n     The size to allocate
   */
  [[nodiscard]] T* allocate(std::size_t n) const {
    if (n == 0) { return nullptr; }
    if (n > std::numeric_limits<size_type>::max() / sizeof(T)) {
      throw bad_alloc(
          "HugePagesAllocator failed: requested more bytes than possible.");
    }

    void* mem =
        ::mmap(nullptr, n * sizeof(T), PROT_READ | PROT_WRITE | PROT_EXEC,
               MAP_HUGETLB | MAP_PRIVATE | MAP_ANONYMOUS, 0, 0);

    if (mem == (void*)(-1)) {
      throw bad_alloc(
          fmt::format("HugePagesAllocator failed: mmap failed with error: {}.",
                      errno != 0 ? std::strerror(errno) : "unknown"));
    }

    if (::madvise(mem, n * sizeof(T), static_cast<int>(Ad))) {
      throw bad_alloc(fmt::format(
          "HugePagesAllocator failed: madvise failed with error: {}.",
          errno != 0 ? std::strerror(errno) : "unknown"));
    }

    if (mem != nullptr) {
      return reinterpret_cast<T*>(mem);
    } else {
      fmt::print("NULL\n");
      throw bad_alloc("HugePagesAllocator failed: null pointer");
    }
  }

  /**
   * @brief      Frees memory
   *
   * @param      p     The pointer to memory to deallocate
   * @param[in]  n     The size to deallocate
   */
  void deallocate(T* p, std::size_t n) const {
    ::munmap(reinterpret_cast<void*>(p), n);
  }
};

/**
 * @brief      Checks if allocators are not the same
 *
 * @param[in]  lhs   The left hand side
 * @param[in]  rhs   The right hand side
 *
 * @tparam     T1    The lhs value type
 * @tparam     Ad1   The lhs advice
 * @tparam     T2    The rhs value type
 * @tparam     Ad2   The rhs advice
 *
 * @return     Always false (stateless allocator)
 */
template <typename T1, Advice Ad1, typename T2, Advice Ad2>
bool operator!=(const HugePagesAllocator<T1, Ad1>& lhs,
                const HugePagesAllocator<T2, Ad2>& rhs) noexcept {
  return false;
}

/**
 * @brief      Checks if allocators are the same
 *
 * @param[in]  lhs   The left hand side
 * @param[in]  rhs   The right hand side
 *
 * @tparam     T1    The lhs value type
 * @tparam     Ad1   The lhs advice
 * @tparam     T2    The rhs value type
 * @tparam     Ad2   The rhs advice
 *
 * @return     Always true (stateless allocator)
 */
template <typename T1, Advice Ad1, typename T2, Advice Ad2>
bool operator==(const HugePagesAllocator<T1, Ad1>& lhs,
                const HugePagesAllocator<T2, Ad2>& rhs) noexcept {
  return true;
}

template <typename T, std::size_t Al = alignof(T), Advice Ad = Advice::normal>
struct AdvisedAlignedAllocator {
  /* Required typedefs/aliases */
  using value_type                             = T;
  using size_type                              = std::size_t;
  using difference_type                        = ptrdiff_t;
  using propagate_on_container_move_assignment = std::true_type;
  using is_always_equal                        = std::true_type;

  /* Constructors */
  AdvisedAlignedAllocator()                                     = default;
  AdvisedAlignedAllocator(const AdvisedAlignedAllocator& other) = default;
  AdvisedAlignedAllocator& operator=(const AdvisedAlignedAllocator& other) =
      delete;
  template <typename U>
  AdvisedAlignedAllocator(
      const AdvisedAlignedAllocator<U, Al, Ad>&) noexcept {};

  ~AdvisedAlignedAllocator() = default;

  /* Deprecated */
  template <class U>
  struct rebind {
    using other = AdvisedAlignedAllocator<U, Al, Ad>;
  };

  /**
   * @brief      Allocates memory
   *
   * @param[in]  n     The size to allocate
   */
  [[nodiscard]] T* allocate(std::size_t n) const {
    if (n == 0) { return nullptr; }
    if (n > std::numeric_limits<size_type>::max() / sizeof(T)) {
      throw bad_alloc(
          "AdvisedAlignedAllocator failed: requested more bytes than "
          "possible.");
    }

    /* Get aligned storage. */
    void* mem = nullptr;
    if (::posix_memalign(&mem, Al, n * sizeof(T)))
      throw bad_alloc(
          fmt::format("AdvisedAlignedAllocator failed: posix_memalign failed "
                      "with error: {}.",
                      errno != 0 ? std::strerror(errno) : "unknown"));

    if (mem == (void*)(-1))
      throw bad_alloc("AdvisedAlignedAllocator failed: bad pointer");

    /* Advise the kernel about the use of the mmap'ed memory. */
    if (::madvise(mem, n * sizeof(T), static_cast<int>(Ad)) != 0)
      throw bad_alloc(fmt::format(
          "AdvisedAlignedAllocator failed: madvise failed with error: {}.",
          errno != 0 ? std::strerror(errno) : "unknown"));

    if (mem != nullptr) {
      return reinterpret_cast<T*>(mem);
    } else {
      throw bad_alloc("AdvisedAlignedAllocator failed: null pointer");
    }
  }

  /**
   * @brief      Frees memory
   *
   * @param      p     The pointer to memory to deallocate
   * @param[in]  n     The size to deallocate (unused)
   */
  void deallocate(T* p, std::size_t n) const {
    std::free(p);
  }
};

/**
 * @brief      Checks if allocators are not the same
 *
 * @param[in]  lhs   The left hand side
 * @param[in]  rhs   The right hand side
 *
 * @tparam     T1    The lhs value type
 * @tparam     Al1   The lhs alignment
 * @tparam     Ad1   The lhs advice
 * @tparam     T2    The rhs value type
 * @tparam     Al1   The rhs alignment
 * @tparam     Ad2   The rhs advice
 *
 * @return     Always false (stateless allocator)
 */
template <typename T1, std::size_t Al1, Advice Ad1,  //
          typename T2, std::size_t Al2, Advice Ad2>
bool operator!=(const AdvisedAlignedAllocator<T1, Al1, Ad1>& lhs,
                const AdvisedAlignedAllocator<T2, Al2, Ad2>& rhs) noexcept {
  return false;
}

/**
 * @brief      Checks if allocators are the same
 *
 * @param[in]  lhs   The left hand side
 * @param[in]  rhs   The right hand side
 *
 * @tparam     T1    The lhs value type
 * @tparam     Al1   The lhs alignment
 * @tparam     Ad1   The lhs advice
 * @tparam     T2    The rhs value type
 * @tparam     Al2   The rhs alignment
 * @tparam     Ad2   The rhs advice
 *
 * @return     Always true (stateless allocator)
 */
template <typename T1, std::size_t Al1, Advice Ad1,  //
          typename T2, std::size_t Al2, Advice Ad2>
bool operator==(const AdvisedAlignedAllocator<T1, Al1, Ad1>& lhs,
                const AdvisedAlignedAllocator<T2, Al2, Ad2>& rhs) noexcept {
  return true;
}

#endif

}  // namespace exot::utilities
