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
 * @file meters/process_android.cpp
 * @author     Philipp Miedl
 * @brief      Implementation of the android foreground process metering module.
 */

#if defined(__linux__) && defined(__ANDROID__) && !defined(__x86_64__)

#include "../../include/exot/meters/process_android.h"
#include <exot/meters/process_android.h>
#include <jni.h>

namespace exot::modules {

process_android::process_android(settings& conf)
    : conf_{validate_settings(conf)} {
}

process_android::~process_android() {
  debug_log_->info("Detaching process measurement thread.");
}

typename process_android::return_type process_android::measure() {
  JavaVM* jvm = reinterpret_cast<JavaVM*>(conf_.jvm);
  // int getEnvStat = conf_.jvm->GetEnv((void**) &jenv_, conf_.jniversion);
  int getEnvStat =
      jvm->GetEnv((void**)&jenv_, static_cast<jint>(conf_.jniversion));
  if (getEnvStat == JNI_EDETACHED) {
    // debug_log_->info( "GetEnv: not attached");
    // if (conf_.jvm->AttachCurrentThread(&jenv_, NULL) != 0) {
    if (jvm->AttachCurrentThread(&jenv_, NULL) != 0) {
      debug_log_->info("Failed to attach");
    }
  } else if (getEnvStat == JNI_OK) {
    //
  } else if (getEnvStat == JNI_EVERSION) {
    debug_log_->info("GetEnv: version not supported");
  }

  // jstring processname = (jstring) (jenv_->CallObjectMethod(*conf_.jinstance,
  // *conf_.jmid));
  jstring processname = (jstring)(
      jenv_->CallObjectMethod(reinterpret_cast<jobject>(conf_.jinstance),
                              reinterpret_cast<jmethodID>(conf_.jmid)));
  std::string procname(jenv_->GetStringUTFChars(processname, 0));

  if (jenv_->ExceptionCheck()) { jenv_->ExceptionDescribe(); }
  // conf_.jvm->DetachCurrentThread();
  jvm->DetachCurrentThread();
  return procname;
}

std::vector<std::string> process_android::header() {
  std::vector<std::string> description;
  description.push_back(exot::utilities::generate_header(
      conf_.name(), "ForegroundActivity", 0, DefaultUnits::process));
  return description;
}

process_android::settings process_android::validate_settings(settings& conf) {
  /* TODO If no JNE pointer is provided, die gracefully... */
  return conf;
}

}  // namespace exot::modules

#endif
