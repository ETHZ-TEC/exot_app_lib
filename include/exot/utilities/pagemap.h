/**
 * @file utilities/pagemap.h
 * @author     Bruno Klopott
 * @brief      Wrapper for Linux virtual-to-physical address resolution via
 *             proc/<pid>/pagemap.
 */

#pragma once

#if defined(__linux__)

#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <thread>
#include <type_traits>

#include <fmt/format.h>

#include <exot/utilities/bits.h>
#include <exot/utilities/types.h>

namespace exot::utilities {

/**
 * @brief      Gets the page size.
 *
 * @return     The page size.
 */
static inline long get_page_size() {
  static auto page_size = ::sysconf(_SC_PAGE_SIZE);
  return page_size;
}

/**
 * @brief      Linux pagemap data
 * @note       In a page is swapped, the page_frame_number field will contain
 *             a value that has bits 0-4: swap type, 5-54: swap offset.
 * @see        Linux documentation at admin-guide/mm/pagemap.rst.
 */
struct PagemapValue {
  std::uint64_t page_frame_number : 54;           // 0-54
  std::uint64_t page_table_entry_soft_dirty : 1;  // 55
  std::uint64_t page_mapped_exclusively : 1;      // 56
  std::uint64_t : 4;                              // 57-60
  std::uint64_t page_is_file_or_anon : 1;         // 61
  std::uint64_t page_swapped : 1;                 // 62
  std::uint64_t page_present : 1;                 // 63
};

/**
 * @brief      Utility functor that translates virtual addresses to physical
 */
struct AddressTranslator {
  /**
   * Constructs the functor with the calling process'es pagemap
   */
  AddressTranslator() : AddressTranslator(std::string{"self"}) {}

  /**
   * @brief      Constructs the functor for a specific PID
   *
   * @param[in]  pid   The pid
   */
  explicit AddressTranslator(pid_t pid)
      : AddressTranslator(std::to_string(static_cast<std::uintmax_t>(pid))) {}

  /**
   * @brief      Destroys the object.
   */
  ~AddressTranslator() { ::close(fd_); }

  /**
   * @brief      Translates a virtual address to a physical address
   *
   * @note       In modern Linux kernel versions, the pagemap can be accessed
   *             only by users with admin capabilities. If the pagemap is read
   *             with insufficient capabilities the returned page frame number
   *             will be 0, and the addresses will no longer be meaningful.
   *
   * @param[in]  virtual_address  The virtual address to convert
   *
   * @tparam     T                the address type (either, void*, a pointer
   *                              type or an integer that can hold a pointer).
   * @tparam     <unnamed>        Template helper
   *
   * @return     The corresponding physical address
   */
  template <typename T,
            typename = std::enable_if_t<
                (std::is_same<T, void*>::value || std::is_pointer<T>::value ||
                 exot::utilities::is_convertible_d<T, std::uintptr_t>::value)>>
  auto operator()(T virtual_address) -> std::uintptr_t {
    std::uintptr_t virtual_address_;

    if constexpr (std::is_same<T, void*>::value || std::is_pointer<T>::value) {
      virtual_address_ = reinterpret_cast<std::uintptr_t>(virtual_address);
    } else {
      virtual_address_ = static_cast<std::uintptr_t>(virtual_address);
    }

    /**
     * @note       Workaround for soft-dirty bit being set upon first iteration.
     *             It seems that accessing the memory first solves the issue.
     * @todo       Is there a more 'proper' way to accomplish the task?
     */
    volatile auto _ = *reinterpret_cast<std::uintptr_t*>(virtual_address);

    auto pv = get_pagemap_value(virtual_address_);

    if (!pv.has_value()) {
      throw std::logic_error(
          "virtual address cannot be translated to physical");
    }

    auto page_frame_number = pv.value().page_frame_number;
    /* TODO: check if page frame returns 0, meaning that the process did not
     * have the capabilities to access the pagemap. */

    return page_frame_number * page_size_ + (virtual_address_ % page_size_);
  }

  /**
   * @brief      Gets the pagemap value for a given address.
   *
   * @param[in]  virtual_address  The virtual address
   *
   * @return     The pagemap value, or nullopt if unsuccessful.
   */
  auto get_pagemap_value(uintptr_t virtual_address)
      -> std::optional<PagemapValue> {
    using namespace exot::utilities;

    std::uint64_t value = 0ull;
    unsigned n          = 0u;

    /* Read sizeof(value) bytes from the pagemap, e.g. 8 on 64-bit CPUs. */
    while (n < sizeof(value)) {
      auto _ = ::pread(fd_, reinterpret_cast<void*>(&value), sizeof(value),
                       static_cast<off_t>(
                           (virtual_address / page_size_) * sizeof(value) + n));
      if (_ <= 0)
        return {};
      else
        n += _;
    }

    PagemapValue pv;

    /* Fill the PagemapValue structure with extracted bits/bit ranges. */
    pv.page_frame_number           = extract_bit_range(value, 0ull, 54ull);
    pv.page_table_entry_soft_dirty = get_bit(value, 55ull);
    pv.page_mapped_exclusively     = get_bit(value, 56ull);
    pv.page_is_file_or_anon        = get_bit(value, 61ull);
    pv.page_swapped                = get_bit(value, 62ull);
    pv.page_present                = get_bit(value, 63ull);

    return std::move(pv);
  }

 private:
  int fd_{-1};             //! the file descriptor to the pagemap
  std::string interface_;  //! the path to the pagemap
  long page_size_{::sysconf(_SC_PAGE_SIZE)};  //! the system's page size

  /**
   * @brief      Non-public constructor using a string path
   *
   * @param[in]  proc_path  The path to the proc path with the pagemap
   */
  AddressTranslator(const std::string& proc_path) {
    interface_ = fmt::format("/proc/{}/pagemap", proc_path);

    if (fd_ = ::open(interface_.c_str(), O_RDONLY); fd_ == -1) {
      throw std::logic_error(
          fmt::format("{} not accessible, reason: {}", interface_,
                      errno != 0 ? std::strerror(errno) : "unknown"));
    }

    // TODO: writing to clear_refs doesn't seem to take care of soft-dirty bits
    // auto cr_ = fmt::format("/proc/{}/clear_refs", proc_path);

    // if (auto clear_refs_ = std::ofstream(cr_, std::ios::out);
    //     !clear_refs_.is_open()) {
    //   throw std::logic_error(fmt::format("{} not accessible", cr_));
    // } else {
    //   clear_refs_ << 4;
    //   clear_refs_.close();
    // }
  }
};

}  // namespace exot::utilities

#else
#error "The pagemap utility can only be used on Linux"
#endif
