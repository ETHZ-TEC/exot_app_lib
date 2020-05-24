/**
 * @file utilities/filesystem.h
 * @author     Bruno Klopott
 * @brief      Functions for listing/grepping directories, and other
 *             filesystem-related activities.
 */

#pragma once

/**
 * Since std::filesystem is not yet mature enough to be included in some STL
 * implementations, notably on Android, we have to rely on more standard Unix
 * methods. The <dirent.h> should be available on most *nix platforms.
 *
 * Globbing via <glob.h> is not available on Android platforms before API level
 * 28, hence <regex>-based alternatives are provided.
 *
 * __has_include macro is part of the C++17 standard. In the case when a
 * platform does not provide the header file, and the contained functions are
 * called, the linker will most likely complain about the lack of defined
 * symbols.
 */
#if __has_include(<dirent.h>)
#include <dirent.h>
#endif

#if __has_include(<ftw.h>)
#include <ftw.h>
#endif

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>    // for remove_if
#include <cerrno>       // for errno
#include <cstring>      // for strerror
#include <fstream>      // for file access
#include <memory>       // for unique_ptr
#include <optional>     // for optional
#include <regex>        // for regular expressions
#include <string>       // for strings
#include <type_traits>  // for type traits
#include <vector>       // for variable-size arrays

#include <fmt/format.h>

#include <exot/utilities/types.h>

namespace exot::utilities {

using stat_t = struct stat;  //! alias for struct stat

/* list_directory and grep_directory require dirent.h */
#if __has_include(<dirent.h>)

/**
 * @brief      Lists the contents of a directory
 * @details    The current and parent directories ('.', and '..') are also
 *             present in the output.
 *
 *             Throws an error if the directory does not exist or is otherwise
 *             unreadable.
 *
 * @param[in]  directory  The directory
 *
 * @return     A vector of strings containing the paths of files and directories
 *             present in the listed directory.
 */
std::vector<std::string> list_directory(const std::string& directory);

/**
 * @brief      Lists a directory and return paths that match a regular
 *             expression.
 *
 * @param[in]  directory  The directory
 * @param[in]  regex      The regular expression
 *
 * @return     A vector of strings containing the paths of files and directories
 *             present in the listed directory that match the regular
 *             expression.
 */
std::vector<std::string> grep_directory(const std::string& directory,
                                        const std::string& regex);

#endif

/**
 * @brief      Enum to indicate whether symlinks are to be followed or ignored.
 */
enum class Symlinks { Follow, Ignore };

/* Recursive listing and grepping requires ftw.h */
#if __has_include(<ftw.h>)

/**
 * @brief      Lists the contents of a directory recursively
 *
 * @param[in]  directory  The directory
 * @param[in]  symlinks   Option to follow/ignore symlinked directories
 *
 * @return     A vector of strings containing the paths of files and directories
 *             present in the listed directory and its subdirectories.
 */
std::vector<std::string> list_directory_r(const std::string& directory,
                                          Symlinks symlinks = Symlinks::Ignore);

/**
 * @brief      Lists a directory recursively and returns paths that match a
 *             regular expression
 *
 * @param[in]  directory  The directory
 * @param[in]  regex      The regular expression
 * @param[in]  symlinks   Option to follow/ignore symlinked directories
 *
 * @return     A vector of strings containing the paths of files and directories
 *             present in the listed directory that match the regular
 *             expression.
 */
std::vector<std::string> grep_directory_r(const std::string& directory,
                                          const std::string& regex,
                                          Symlinks symlinks = Symlinks::Ignore);

#endif

/**
 * @brief      Checks if a provided file can be read
 *
 * @param[in]  path  The path
 *
 * @return     True if readable, False otherwise.
 */
bool is_readable(const std::string& path);

/**
 * @brief      Checks if a provided path exists
 *
 * @param[in]  path  The path
 *
 * @return     True if exists, False otherwise.
 */
bool exists(const std::string& path);

/**
 * @brief      Determines if a path is a directory
 *
 * @param[in]  path  The path
 *
 * @return     True if directory, False otherwise.
 */
bool is_directory(const std::string& path);

/**
 * @brief      Gets the sys/stat.h 'stat' struct for a path
 *
 * @param[in]  path  The path
 *
 * @return     The stat if successful, nullopt otherwise.
 */
auto get_stat(const std::string& path) -> std::optional<stat_t>;

/**
 * @brief      Checks if all provided files can be accessed
 *
 * @param[in]  paths  A vector of path names
 *
 * @return     True if all are readable
 */
bool all_readable(const std::vector<std::string>& paths);

/**
 * @brief      Determines if a file is empty.
 *
 * @param[in]  file_path  The file path
 *
 * @return     True if empty, False otherwise.
 */
bool is_empty(const std::string& file_path);

/**
 * @brief      Removes unreadable files from a vector of paths
 *
 * @param      paths  The vector of path names
 *
 * @return     A vector of paths that have been removed, can be empty.
 */
std::vector<std::string> remove_unreadable(std::vector<std::string>& paths);

/**
 * @brief      Get a value of certain type from file.
 *
 * The function template relies on input stream overloads being present for
 * the type defined in template parameters.
 *
 * @param[in]  path  The path
 *
 * @tparam     To    The desired type
 *
 * @return     The value from file if readable, nullopt otherwise.
 */
template <typename To>
inline auto get_value_from_file(const std::string& path) -> std::optional<To> {
  try {
    if (is_readable(path)) {
      To output;
      std::ifstream file{path};
      file >> output;
      return output;
    } else {
      return {};
    }
  } catch (...) { return {}; }
}

/**
 * @brief      Gets the long string value from file.
 *
 * @param[in]  path  The path as a const string reference
 *
 * @return     The long string value from file if readable, nullopt otherwise.
 */
auto get_long_string_value_from_file(const std::string& path) noexcept
    -> std::optional<std::string>;

/**
 * @brief      Gets a string value from a file
 *
 * @param[in]  path  The path as a const string reference
 *
 * @return     The string value from file if readable, nullopt otherwise.
 */
auto get_string_value_from_file(const std::string& path) noexcept
    -> std::optional<std::string>;

/**
 * @brief      File reader with raw data access
 */
struct file_reader {
  explicit file_reader(std::string&& path) : path_{path} {
    if (fd_ = ::open(path_.c_str(), O_RDONLY); fd_ == -1) {
      throw std::logic_error(
          fmt::format("{} not accessible, reason: {}", path_,
                      errno != 0 ? std::strerror(errno) : "unknown"));
    }

    if (!is_readable(path_)) {
      throw std::logic_error(fmt::format("{} not accessible", path_));
    }
  }

  file_reader& operator=(file_reader&& other) {
    fd_ = std::move(other.fd_);
    return *this;
  }

  ~file_reader() { ::close(fd_); }

  /**
   * @brief      Read from file into a data type T
   *
   * @tparam     T          The return data type
   * @tparam     <unnamed>  Template helper
   *
   * @return     The read value if successful, nullopt otherwise.
   */
  template <typename T, typename = std::enable_if_t<
                            std::is_default_constructible<T>::value>>
  auto read_raw() -> std::optional<T> {
    auto value      = T{};
    auto bytes_read = 0u;

    while (bytes_read < sizeof(value)) {
      auto _ = ::pread(fd_, reinterpret_cast<void*>(&value), sizeof(value),
                       static_cast<off_t>(bytes_read));
      if (_ <= 0)
        return {};
      else
        bytes_read += _;
    }

    return value;
  }

  template <typename T, typename = std::enable_if_t<
                            (std::is_default_constructible<T>::value &&
                             is_readable_from_stream<T>::value)>>
  auto read_stream() -> std::optional<T> {
    auto value = T{};
    try {
      auto file = std::ifstream{path_};
      file >> value;
      return value;
    } catch (...) { return {}; }
  }

 private:
  int fd_;
  std::string path_;
};

}  // namespace exot::utilities
