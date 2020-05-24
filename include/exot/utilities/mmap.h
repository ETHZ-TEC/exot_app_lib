/**
 * @file utilities/mmap.h
 * @author     Bruno Klopott
 * @brief      Wrapper for Linux memory mapped files
 */

#pragma once

#if defined(__linux__)

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <initializer_list>

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <exot/utilities/filesystem.h>
#include <exot/utilities/helpers.h>
#include <exot/utilities/types.h>

namespace exot::utilities {

/**
 * @brief      Enumeration class for mmap memory protection argument
 */
enum class MMapProt : int {
  Exec  = PROT_EXEC,
  Read  = PROT_READ,
  Write = PROT_WRITE,
  None  = PROT_NONE
};

/**
 * JSON serialisation for MMapProt
 */
NLOHMANN_JSON_SERIALIZE_ENUM(MMapProt, {
                                           {MMapProt::Exec, "Exec"},
                                           {MMapProt::Read, "Read"},
                                           {MMapProt::Write, "Write"},
                                           {MMapProt::None, "None"},
                                       })

#if !defined(MAP_HUGE_2MB) && defined(MAP_HUGE_SHIFT)
#define MAP_HUGE_2MB (21 << MAP_HUGE_SHIFT)
#endif

#if !defined(MAP_HUGE_1GB) && defined(MAP_HUGE_SHIFT)
#define MAP_HUGE_1GB (30 << MAP_HUGE_SHIFT)
#endif

/**
 * @brief      Enumeration class for reasonable mmap flags
 * @note       Some flags might not be available, depending on how the kernel
 *             was compiled. Some of these options are commented out below in
 *             the enum and the macro serialisation mapping.
 */
enum class MMapFlag : int {
  Shared    = MAP_SHARED,
  Private   = MAP_PRIVATE,
  Anonymous = MAP_ANONYMOUS,
  Fixed     = MAP_FIXED,
  GrowsDown = MAP_GROWSDOWN,
  HugeTable = MAP_HUGETLB,
#if defined(MAP_HUGE_2MB)
  HugeTable2MB = MAP_HUGETLB | MAP_HUGE_2MB,
#endif
#if defined(MAP_HUGE_1GB)
  HugeTable1GB = MAP_HUGETLB | MAP_HUGE_1GB,
#endif
  Locked           = MAP_LOCKED,
  Populate         = MAP_POPULATE,
  PopulateNonBlock = MAP_POPULATE | MAP_NONBLOCK,
  NoReserve        = MAP_NORESERVE,
  Stack            = MAP_STACK,
#if defined(MAP_SHARED_VALIDATE)
  SharedValidate = MAP_SHARED_VALIDATE,
#endif
#if defined(MAP_SHARED_VALIDATE) && defined(MAP_SYNC)
  SharedValidateSync = MAP_SHARED_VALIDATE | MAP_SYNC,
#endif
#if defined(MAP_FIXED_NOREPLACE)
  FixedNoReplace = MAP_FIXED_NOREPLACE,
#endif
#if defined(MAP_UNINITIALIZED)
  Uninitialized = MAP_UNINITIALIZED,
#endif
};

/**
 * JSON serialisation form MMapFlag
 */
NLOHMANN_JSON_SERIALIZE_ENUM(MMapFlag, {
  {MMapFlag::Shared, "Shared"}, {MMapFlag::Private, "Private"},
      {MMapFlag::Anonymous, "Anonymous"}, {MMapFlag::Fixed, "Fixed"},
      {MMapFlag::GrowsDown, "GrowsDown"}, {MMapFlag::HugeTable, "HugeTable"},
#ifdef MAP_HUGE_2MB
      {MMapFlag::HugeTable2MB, "HugeTable2MB"},
#endif
#ifdef MAP_HUGE_1GB
      {MMapFlag::HugeTable1GB, "HugeTable1GB"},
#endif
      {MMapFlag::Locked, "Locked"}, {MMapFlag::Populate, "Populate"},
      {MMapFlag::PopulateNonBlock, "PopulateNonBlock"},
      {MMapFlag::NoReserve, "NoReserve"}, {MMapFlag::Stack, "Stack"},
#if defined(MAP_SHARED_VALIDATE)
      {MMapFlag::SharedValidate, "SharedValidate"},
#endif
#if defined(MAP_SHARED_VALIDATE) && defined(MAP_SYNC)
      {MMapFlag::SharedValidateSync, "SharedValidateSync"},
#endif
#if defined(MAP_FIXED_NOREPLACE)
      {MMapFlag::FixedNoReplace, "FixedNoReplace"},
#endif
#if defined(MAP_UNINITIALIZED)
      {MMapFlag::Uninitialized, "Uninitialized"},
#endif
})

/**
 * @brief      Wrapper for mapping files/devices into memory via mmap
 */
class MMap {
 public:
  using addr_type   = void*;   //! 1: starting address for the mapping
  using length_type = size_t;  //! 2: length of the mapping (>0)
  using prot_type   = int;     //! 3: memory protection mode
  using flags_type  = int;     //! 4: mmap flags
  using fd_type     = int;     //! 5: file descriptor
  using offset_type = off_t;   //! 6: mapping offset (multiple of page size)

  /**
   * @brief      No default constructor
   */
  MMap() = delete;

  /**
   * @brief      Map a file into the program's memory.
   *
   * @param[in]  filename          The filename
   * @param[in]  starting_address  The starting address
   * @param[in]  length            The length of the mapping
   * @param[in]  prots             The mapping's memory protection modes
   * @param[in]  flags             The flags to pass to mmap
   * @param[in]  offset            The starting offset of file mapping
   */
  explicit MMap(const std::string& filename,            //
                addr_type starting_address,             //
                length_type length,                     //
                std::initializer_list<MMapProt> prots,  //
                std::initializer_list<MMapFlag> flags,  //
                offset_type offset)
      : filename_{filename}, length_{length} {
    using namespace exot::utilities;

    /* Determine if the mapping is meant to be anonymous, i.e. not backed by
     * any file. */
    anonymous_ = contains(MMapFlag::Anonymous, flags);

    /* Compute flags and the memory protection mode. */
    for (auto prot : prots) prot_ |= to_underlying_type(prot);
    for (auto flag : flags) flags_ |= to_underlying_type(flag);

    /* If not anonymous... */
    if (!anonymous_) {
      /* ...Check if the path exists. */
      if (!exists(filename_)) {
        throw std::logic_error(fmt::format(
            "file {} does not exist and cannot be memory mapped", filename_));
      }

      /* ...Determine the file access mode. */
      auto open_mode = O_RDONLY;
      if (prot_ == to_underlying_type(MMapProt::Write)) {
        open_mode = O_WRONLY;
      } else if (prot_ == to_underlying_type(MMapProt::Read) |
                 to_underlying_type(MMapProt::Write)) {
        open_mode = O_RDWR;
      }

      /* ...Get a file descriptor. */
      if (fd_ = ::open(filename_.c_str(), open_mode); fd_ == -1) {
        throw std::logic_error(
            fmt::format("could not open file {}", filename_));
      }

      /* ...Get the file stat, especially the size in bytes. */
      struct stat stat_result;
      if (::fstat(fd_, &stat_result) == -1) {
        throw std::logic_error(
            fmt::format("fstat failed on file {}", filename_));
      }

      /* ...If length of 0 (or no length) is provided, map the entire file. */
      if (length_ == 0) {
        length_ = stat_result.st_size;
      } else if (length_ > stat_result.st_size) {
        throw std::logic_error(
            fmt::format("cannot create mapping of length {} > file size ({} B)",
                        length_, stat_result.st_size));
      }
    } /* If anonymous... */ else { fd_ = 0; }

    /* Check if offset is a multiple of the page size. */
    if (offset_ = offset; offset_ % page_size_ != 0) {
      throw std::logic_error(
          fmt::format("offset ({}) must be a multiple of page size ({})",
                      offset_, page_size_));
    }

    if (anonymous_ && offset_ != 0) {
      throw std::logic_error("anonymous mapping did not have a 0 offset");
    }

    /* Call the unix mmap function. */
    addr_ = ::mmap(starting_address, length_, prot_, flags_, fd_, offset_);

    /* Close the file descriptor. */
    if (fd_ != -1) ::close(fd_);

    if (addr_ == (void*)(-1) || addr_ == static_cast<addr_type>(0)) {
      throw std::logic_error(
          fmt::format("mmap failed{}, reason: {}",
                      anonymous_ ? "" : fmt::format(" on file {}", filename_),
                      errno != 0 ? std::strerror(errno) : "unknown"));
    }
  }

  /**
   * @brief      Delegated constructor which maps the entire file
   *
   * @param[in]  filename  The filename
   * @param[in]  prots     The mapping's memory protection modes
   * @param[in]  flags     The flags to pass to mmap
   */
  explicit MMap(const std::string& filename,
                std::initializer_list<MMapProt> prots,
                std::initializer_list<MMapFlag> flags)
      : MMap(filename, nullptr, 0, prots, flags, 0) {}

  /**
   * @brief      Delegated constructor for anonymous mappings
   *
   * @param[in]  length  The length of the mapping (in bytes)
   * @param[in]  prots   The mapping's memory protection modes
   * @param[in]  flags   The flags to pass to mmap
   */
  explicit MMap(length_type length,  //
                std::initializer_list<MMapProt> prots,
                std::initializer_list<MMapFlag> flags)
      : MMap(std::string{}, nullptr, length, prots, flags, 0) {}

  /**
   * @brief      Destroys the object.
   */
  ~MMap() {
    /* Unmap the memory. */
    auto _ = ::munmap(addr_, length_);
  }

  /*
   * Getters
   */

  // clang-format off
  addr_type   get_address()   const { return addr_; }
  length_type get_length()    const { return length_; }
  prot_type   get_prot()      const { return prot_; }
  flags_type  get_flags()     const { return flags_; }
  fd_type     get_fd()        const { return fd_; }
  offset_type get_offset()    const { return offset_; }
  offset_type get_page_size() const { return page_size_; }
  // clang-format on

  /*
   * Other
   */

  bool is_anonymous() { return anonymous_; }

 private:
  addr_type addr_{nullptr};  //! the address returned by mmap
  length_type length_{0};    //! the length of the mapping
  prot_type prot_{0};        //! the mmap protection mode
  flags_type flags_{0};      //! the mmap flags
  fd_type fd_{-1};           //! the mmap file descriptor
  offset_type offset_{0};    //! the mapping offset
  std::string filename_;     //! the file to map, empty by default
  long page_size_{::sysconf(_SC_PAGE_SIZE)};  //! the system's page size
  bool anonymous_{false};                     //! is the mapping anonymous?
};

}  // namespace exot::utilities

#endif
