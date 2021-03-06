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
 * @file meters/process_android.h
 * @author     Philipp Miedl
 * @brief      Current Process meter module for Android.
 */

#pragma once

#if defined(__linux__) && defined(__ANDROID__)

#include <jni.h>  // for Java Native Interface access

#include <algorithm>  // for std::transform
#include <cstdint>    // for std::uint*
#include <fstream>    // for ifstream
#include <string>     // for std::string
#include <thread>     // for hardware_concurrency
#include <vector>     // for variable-size arrays

#include <fmt/format.h>     // for formatting strings
#include <fmt/ostream.h>    // for ostream support
#include <spdlog/spdlog.h>  // for logging

#include <spdlog/sinks/stdout_color_sinks.h>

#include <exot/meters/base.h>
#include <exot/utilities/bits.h>           // for bit manipulation functions
#include <exot/utilities/configuration.h>  // for configurable
#include <exot/utilities/ostream.h>        // for ostream operator overloads

namespace exot::modules {

struct process_android : module {
  using return_type = std::string;

  struct settings : public exot::utilities::configurable<settings> {
    // jclass* jclazz;
    // jobject* jinstance;
    // jmethodID* jmid;
    // JavaVM* jvm;
    // jint jniversion;
    // bool dummy{false};

    std::uintptr_t jclazz;
    std::uintptr_t jinstance;
    std::uintptr_t jmid;
    std::uintptr_t jvm;
    int jniversion;
    bool dummy{false};

    const char* name() const { return "process_android"; }

    void configure() {
      bind_and_describe_data("jclazz", jclazz);
      bind_and_describe_data("jinstance", jinstance);
      bind_and_describe_data("jmid", jmid);
      bind_and_describe_data("jvm", jvm);
      bind_and_describe_data("jniversion", jniversion);
      bind_and_describe_data("dummy", dummy);
    }
  };

  explicit process_android(settings& conf);

  ~process_android();

  return_type measure();

  std::vector<std::string> header();

 private:
  using logger_pointer = std::shared_ptr<spdlog::logger>;
  logger_pointer debug_log_ =
      spdlog::get("log") ? spdlog::get("log") : spdlog::stderr_color_mt("log");

  settings conf_;
  return_type readings;
  JNIEnv* jenv_;

  /**
   * @brief      Provide sensible settings
   * @details    The used Java Native Environment pointer is checked here.
   *             If its not valid, the service should die gracefully.
   *
   * @param      conf  Module settings
   *
   * @return     Module settings
   */
  settings validate_settings(settings&);
};

}  // namespace exot::modules

#endif
