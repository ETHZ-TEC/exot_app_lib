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
#include <exot/generators/utilisation_ut.h>

#include <chrono>
#include <thread>

#include <spdlog/spdlog.h>

using namespace exot::modules;

void generator_utilisation_ut::generate_load(
    const generator_utilisation_ut::decomposed_type& decomposed_subtoken,
    const generator_utilisation_ut::enable_flag_type& flag,
    generator_utilisation_ut::core_type core,
    generator_utilisation_ut::index_type index) {
  volatile double a = 2.0e0;
  volatile double b = 2.0e0;

  if (decomposed_subtoken > 0) {
    SPDLOG_LOGGER_TRACE(logger_,
                        "[generator->generate_load] worker on core {} enabled "
                        "with a utilisation of {}",
                        core, decomposed_subtoken);
    auto active = std::chrono::nanoseconds{10000 * decomposed_subtoken / 100};
    auto wsleep =
        std::chrono::nanoseconds{10000 * (100 - decomposed_subtoken) / 100};
    /* Heavy floating-point work loop. */
    while (flag) {
      for (auto start = std::chrono::steady_clock::now();
           active >= (std::chrono::steady_clock::now() - start);) {
        a *= b;
        b -= a;
      }
      std::this_thread::sleep_for(wsleep);
    }
  }
}
