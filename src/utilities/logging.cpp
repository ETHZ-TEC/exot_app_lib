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
 * @file utilities/logging.cpp
 * @author     Bruno Klopott
 * @brief      Implementation of the core logging configurator from @ref
 *             logging.h.
 */

#include <exot/utilities/logging.h>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <type_traits>
#include <vector>

#include <spdlog/async.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/stdout_sinks.h>

#ifdef __ANDROID__
#include <spdlog/sinks/android_sink.h>
#endif

#include <exot/utilities/formatting.h>
#include <exot/utilities/helpers.h>
#include <exot/utilities/ostream.h>
#include <exot/utilities/platform_id.h>

/* TODO: create a macro to reduce conditional statements' depth */

namespace exot::utilities {

void append_to_filename(std::string& file_path, const std::string& to_append) {
  using clock = std::chrono::system_clock;
  using exot::utilities::join_strings;
  using exot::utilities::split_string;

  /* Split the file path to get the basename */
  auto split_file_path = split_string(file_path, '/');

  /* Split the file name and extension */
  auto split_file_name = split_string(split_file_path.back(), '.');
  auto size            = split_file_name.size();

  /* Append string to the file */
  if (size < 2) {
    split_file_path.back() = split_file_name.at(size - 1) + "_" + to_append;
  } else {
    split_file_path.back() = split_file_name.at(size - 2) + "_" + to_append +
                             "." + split_file_name.at(size - 1);
  }

  /* Join the split path back together */
  file_path = join_strings(split_file_path, '/');
}

void append_timestamp_to_filename(std::string& file_path) {
  using clock = std::chrono::system_clock;

  /* Get the current wall clock time and format it to a string*/
  auto time      = clock::to_time_t(clock::now());
  auto timestamp = fmt::format("{:%Y-%m-%d_%H-%M-%S}", *std::localtime(&time));

  append_to_filename(file_path, timestamp);
}

Logging::Logging(settings& conf) : conf_{conf} {
  /* Remove the "app" and "log" loggers from the registry */
  spdlog::drop("app");
  spdlog::drop("log");

  if (conf_.async) {
    if (!is_power_of_2(conf_.async_size))
      throw std::invalid_argument(
          fmt::format("[{}:{}] async_size has to be non-zero and a power of 2. "
                      "Value of {} was provided.",
                      __FUNCTION__, __LINE__, conf_.async_size));

    spdlog::init_thread_pool(conf_.async_size, conf_.async_thread_count);
  }

  /* Application log -------------------------------------------------------- */

  if (conf_.app_log_filename.has_value() &&
      !conf_.app_log_filename.value().empty()) {
    /* If a filename was provided */

    if (conf_.append_governor_to_files) {
      auto governor = exot::utilities::get_scaling_governor(0);
      if (governor) {
        append_to_filename(conf_.app_log_filename.value(), governor.value());
      }
    }

    if (conf_.timestamp_files) {
      append_timestamp_to_filename(conf_.app_log_filename.value());
    }

    if (conf_.rotating_logs) {
      if (conf_.async) {
        spdlog::rotating_logger_st<spdlog::async_factory>(
            "app", conf_.app_log_filename.value(), conf_.rotating_logs_size,
            conf_.rotating_logs_count);
      } else {
        spdlog::rotating_logger_st<spdlog::default_factory>(
            "app", conf_.app_log_filename.value(), conf_.rotating_logs_size,
            conf_.rotating_logs_count);
      }
    } else {
      if (conf_.async) {
        spdlog::basic_logger_st<spdlog::async_factory>(
            "app", conf_.app_log_filename.value());
      } else {
        spdlog::basic_logger_st<spdlog::default_factory>(
            "app", conf_.app_log_filename.value());
      }
    }
  } else {
    /* If no filename was provided */
#if defined(__ANDROID__)
    if (conf_.async) {
      spdlog::android_logger_st<spdlog::async_factory>("app", "app");
    } else {
      spdlog::android_logger_st<spdlog::default_factory>("app", "app");
    }
#else
    if (conf_.async) {
      spdlog::stdout_logger_st<spdlog::async_factory>("app");
    } else {
      spdlog::stdout_logger_st<spdlog::default_factory>("app");
    }
#endif
  }

  /* configure loglevel and pattern */
  spdlog::get("app")->set_level(spdlog::level::trace);
  spdlog::get("app")->set_pattern("%v");

  /* Debug log -------------------------------------------------------------- */

  if (conf_.debug_log_filename.has_value() &&
      !conf_.debug_log_filename.value().empty()) {
    /* If a filename was provided */

    if (conf_.append_governor_to_files) {
      auto governor = exot::utilities::get_scaling_governor(0);
      if (governor) {
        append_to_filename(conf_.debug_log_filename.value(), governor.value());
      }
    }

    if (conf_.timestamp_files) {
      append_timestamp_to_filename(conf_.debug_log_filename.value());
    }

    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(std::make_shared<spdlog::sinks::basic_file_sink_mt>(
        conf_.debug_log_filename.value()));
#if defined(__ANDROID__)
    sinks.push_back(std::make_shared<spdlog::sinks::android_sink_mt>());
#else
    sinks.push_back(
        std::make_shared<spdlog::sinks::ansicolor_stderr_sink_mt>());
#endif
    if (conf_.async) {
      spdlog::register_logger(std::make_shared<spdlog::async_logger>(
          "log", sinks.begin(), sinks.end(), spdlog::thread_pool(),
          spdlog::async_overflow_policy::block));
    } else {
      spdlog::register_logger(
          std::make_shared<spdlog::logger>("log", sinks.begin(), sinks.end()));
    }
  } else {
    /* If no filename was provided */
#if defined(__ANDROID__)
    if (conf_.async) {
      spdlog::android_logger_mt<spdlog::async_factory>("log", "log");
    } else {
      spdlog::android_logger_mt<spdlog::default_factory>("log", "log");
    }
#else
    if (conf_.async) {
      spdlog::stderr_color_mt<spdlog::async_factory>("log");
    } else {
      spdlog::stderr_color_mt<spdlog::default_factory>("log");
    }
#endif
  }

  spdlog::get("log")->set_level(conf_.log_level);
#if !defined(SPDLOG_NO_THREAD_ID)
  spdlog::get("log")->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] [%t] %v");
#else
  spdlog::get("log")->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] %v");
#endif
  spdlog::get("log")->flush_on(spdlog::level::err);

  spdlog::get("log")->info(
      "[Logging] using app logfile: {}, debug logfile: {}",
      conf_.app_log_filename.has_value() ? conf_.app_log_filename.value()
                                         : "none",
      conf_.debug_log_filename.has_value() ? conf_.debug_log_filename.value()
                                           : "none");
  if (conf_.rotating_logs) {
    spdlog::get("log")->info("[Logging] rotating logs, size: {}, count: {}",
                             conf_.rotating_logs_size,
                             conf_.rotating_logs_count);
  }

  if (conf_.provide_platform_identification) {
    spdlog::get("log")->info("[Platform] target architecture: \"{}\"",
                             exot::utilities::get_target_architecture());

    spdlog::get("log")->info(
        "[Platform] threads: {}, cores: {}, sockets: {}, online: [{}]",
        exot::utilities::get_thread_count(), exot::utilities::get_core_count(),
        exot::utilities::get_socket_count(),
        exot::utilities::get_online_cpus_array());

    for (unsigned idx{0}; idx < exot::utilities::get_cpu_count(); ++idx) {
      spdlog::get("log")->info(
          "[Platform] cpu: {}, online: {}, frequencies: [{}], governor: \"{}\"",
          idx, exot::utilities::is_cpu_online(idx),
          exot::utilities::get_cpu_frequencies(idx).value_or(
              std::array<unsigned long, 2>{}),
          exot::utilities::get_scaling_governor(idx).value_or(""));
    }

    for (const auto& model : exot::utilities::get_unique_cpu_models().value_or(
             std::vector<std::string>{})) {
      spdlog::get("log")->info("[Platform] cpu models: {}", model);
    }

    spdlog::get("log")->info("[Platform] topology: {}",
                             exot::utilities::get_complete_topology());

    spdlog::get("log")->info(
        "[Platform] kernel: {}",
        exot::utilities::get_kernel_version().value_or("n/a"));
  }
}

}  // namespace exot::utilities
