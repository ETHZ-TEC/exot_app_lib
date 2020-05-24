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
 * @file utilities/filesystem.cpp
 * @author     Bruno Klopott
 * @brief      Implementation of the filesystem listing and access utilities
 *             from @ref filesystem.h.
 */

#include <exot/utilities/filesystem.h>

#if __has_include(<dirent.h>)

std::vector<std::string> exot::utilities::list_directory(
    const std::string& directory) {
  DIR* current_directory;
  struct dirent* entry;

  std::vector<std::string> output;

  if ((current_directory = opendir(directory.c_str())) != NULL) {
    /* print all the files and directories within directory */
    while ((entry = readdir(current_directory)) != NULL) {
      output.push_back(directory + ((directory.back() != '/') ? "/" : "") +
                       std::string(entry->d_name));
    }
    closedir(current_directory);
  } else {
    throw std::logic_error("Could not access the directory");
  }

  std::sort(output.begin(), output.end());
  return output;
}

std::vector<std::string> exot::utilities::grep_directory(
    const std::string& directory, const std::string& regex) {
  std::vector<std::string> list = list_directory(directory);

  std::regex matcher(regex);

  list.erase(std::remove_if(list.begin(), list.end(),
                            [&matcher](std::string element) -> bool {
                              return !std::regex_match(element, matcher);
                            }),
             list.end());

  std::sort(list.begin(), list.end());
  return list;
}

#endif

#if __has_include(<ftw.h>)

std::vector<std::string> exot::utilities::list_directory_r(
    const std::string& directory, Symlinks symlinks) {
  /* @note       Why are the variables below declared static and why unique
   *             pointers to them are created?
   *
   * This 'trick' is required to make it possible for the lambda to be passed as
   * function pointer to the `nftw` function. Not all closure types can
   * converted or statically casted to a pointer to a function. Specifically,
   * the C++ standard states [expr.prim.lambda.closure:7] that "The closure type
   * for a non-generic lambda-expression with no lambda-capture whose
   * constraints (if any) are satisfied has a conversion function to pointer to
   * function with C++ language linkage having the same parameter and return
   * types as the closure type's function call operator."
   *
   * Therefore it is not possible to directly use a capturing lambda with a
   * C-like function pointer. While a wrapper, and even a generic wrapper can be
   * provided via a trick where we define a static lambda without captures that
   * forwards same arguments to the capturing lambda, it is more straightforward
   * to declare variables to have static storage duration instead of capturing.
   * However, to achieve the expected original behaviour of we would need to
   * reset the static variable to its original value upon every function entry.
   * Below, we declare as static only a unique pointer to a type, and assign it
   * a unique value every time. Unique pointers should take care for us of
   * storage duration and object destruction.
   */
  static std::unique_ptr<std::vector<std::string>> output;
  output = std::make_unique<std::vector<std::string>>();
  static std::unique_ptr<std::string> dir;
  dir = std::make_unique<std::string>(directory);

  auto pusher = [](const char* path, const struct stat*, int,
                   struct FTW*) -> int {
    output->emplace_back(path);
    return 0;
  };

  int flags = 0;

  /* Do not follow symbolic links (directories). `ftw` will not report any file
   * twice, which can lead to some possibly unexpected behaviour, where a fileis
   * reported in the symlinked directory, but not in the symlink's target. For
   * example, in a directory structure:
   *
   * .
   * |_ a
   *    |_a.txt
   *    |_b.txt
   * |_ b -> a // symbolic link
   *
   * file tree walk may produce output:
   *
   * ./b
   * ./b/a.txt
   * ./b/b.txt
   *
   */
  if (symlinks == Symlinks::Ignore) flags |= FTW_PHYS;

  /* The third argument states the maximum number of file descriptors that
   * `nftw` can have opened at any time.
   */
  ::nftw(directory.c_str(), pusher, 20, flags);

  std::sort(output->begin(), output->end());
  return *output;
}

std::vector<std::string> exot::utilities::grep_directory_r(
    const std::string& directory, const std::string& regex, Symlinks symlinks) {
  std::vector<std::string> list = list_directory_r(directory);

  std::regex matcher(regex);

  list.erase(std::remove_if(list.begin(), list.end(),
                            [&matcher](std::string element) -> bool {
                              return !std::regex_match(element, matcher);
                            }),
             list.end());

  std::sort(list.begin(), list.end());
  return list;
}

#endif

bool exot::utilities::is_readable(const std::string& path) {
  std::ifstream file{path};
  return file.is_open();
}

bool exot::utilities::exists(const std::string& path) {
  std::ifstream file{path};
  return file.good();
}

bool exot::utilities::is_directory(const std::string& path) {
  struct stat stat_result;
  if (::stat(path.c_str(), &stat_result) == -1) {
    return false;
  } else if (S_ISDIR(stat_result.st_mode)) {
    return true;
  } else {
    return false;
  }
}

auto exot::utilities::get_stat(const std::string& path)
    -> std::optional<struct stat> {
  struct stat stat_result;
  if (::stat(path.c_str(), &stat_result) == -1) {
    return {};
  } else {
    return stat_result;
  }
}

bool exot::utilities::all_readable(const std::vector<std::string>& paths) {
  return std::all_of(paths.begin(), paths.end(),
                     [](const auto& path) { return is_readable(path); });
}

bool exot::utilities::is_empty(const std::string& file_path) {
  std::ifstream file(file_path);
  return file.peek() == std::ifstream::traits_type::eof();
}

std::vector<std::string> exot::utilities::remove_unreadable(
    std::vector<std::string>& paths) {
  std::vector<std::string> removed;

  for (auto it = paths.begin(); it != paths.end();) {
    if (!is_readable(*it)) {
      removed.push_back(*it);
      it = paths.erase(it);
    } else {
      ++it;
    }
  }

  return removed;
}

auto exot::utilities::get_long_string_value_from_file(
    const std::string& path) noexcept -> std::optional<std::string> {
  try {
    if (is_readable(path)) {
      std::string output;
      std::string tmp;
      std::ifstream file{path};

      while (file >> tmp) { output += tmp + " "; }

      return output;
    } else {
      return {};
    }
  } catch (...) { return {}; }
}

auto exot::utilities::get_string_value_from_file(
    const std::string& path) noexcept -> std::optional<std::string> {
  try {
    if (is_readable(path)) {
      std::string output;
      std::ifstream file{path};
      file >> output;
      return output;
    } else {
      return {};
    }
  } catch (...) { return {}; }
}
