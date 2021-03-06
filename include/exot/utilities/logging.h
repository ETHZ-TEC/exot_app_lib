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
 * @file utilities/logging.h
 * @author     Bruno Klopott
 * @brief      Main logging configuration class.
 */

#pragma once

#include <optional>
#include <string>

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fmt/time.h>
#include <spdlog/spdlog.h>

#include <exot/utilities/configuration.h>  // for configurable

namespace exot::utilities {

/**
 * @brief      Appends a string to the filename.
 *
 * @param      file_path  The file path
 * @param[in]  to_append  To string to append
 */
void append_to_filename(std::string& file_path, const std::string& to_append);

/**
 * @brief      Appends a timestamp to a filename.
 *
 * @param      file_path  The path to the file
 */
void append_timestamp_to_filename(std::string& file_path);

/**
 * @brief      Class that encapsulates all logging options and is responsible
 *             for creating loggers.
 */
class Logging {
 public:
  using logger_pointer = std::shared_ptr<spdlog::logger>;

  struct settings : public exot::utilities::configurable<settings> {
    std::optional<std::string> debug_log_filename;
    std::optional<std::string> app_log_filename;
    spdlog::level::level_enum log_level{spdlog::level::info};
    size_t async_size{1024 * 8};
    bool async{false};
    size_t async_thread_count{1};
    bool timestamp_files{false};
    bool rotating_logs{false};
    bool provide_platform_identification{true};
    bool append_governor_to_files{false};
    size_t rotating_logs_size{1024 * 1024 * 100}; /* 100 MiB */
    size_t rotating_logs_count{10};

    const char* name() const { return "logging"; }

    /* @brief The JSON configuration function */
    void configure() {
      bind_and_describe_data("debug_log_filename", debug_log_filename,
                             "the debug log filename |str|, optional");
      bind_and_describe_data("app_log_filename", app_log_filename,
                             "the app log filename |str|, optional");
      bind_and_describe_data(
          "log_level", log_level,
          "the log level |str|, one of \"trace\", \"debug\", "
          "\"info\", \"warn\", \"err\", \"critical\", \"off\"");
      bind_and_describe_data("async_size", async_size,
                             "async logger buffer size |uint|, power of 2");
      bind_and_describe_data("async", async, "use the async logger? |bool|");
      bind_and_describe_data("async_thread_count", async_thread_count,
                             "async logger thread count |uint|");
      bind_and_describe_data("timestamp_files", timestamp_files,
                             "should timestamp the log files? |bool|");
      bind_and_describe_data("rotating_logs", rotating_logs,
                             "should rotate the logs? |bool|");
      bind_and_describe_data("provide_platform_identification",
                             provide_platform_identification,
                             "should provide platform info? |bool|");
      bind_and_describe_data(
          "append_governor_to_files", append_governor_to_files,
          "should append the frequency governor to filenames? |bool|");
      bind_and_describe_data("rotating_logs_size", rotating_logs_size,
                             "the size of rotating logs in MiB |uint|");
      bind_and_describe_data(
          "rotating_logs_count", rotating_logs_count,
          "the maximum number of rotating logs to keep |uint|");
    }
  };

  Logging(settings& conf);

 private:
  settings conf_;
};

}  // namespace exot::utilities
