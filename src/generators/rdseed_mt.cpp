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
#if defined(__x86_64__)

#include <exot/generators/rdseed_mt.h>

#include <spdlog/spdlog.h>

#include <exot/primitives/rng.h>

using namespace exot::modules;

void generator_rdseed_mt::generate_load(
    const generator_rdseed_mt::decomposed_type& decomposed_subtoken,
    const generator_rdseed_mt::enable_flag_type& flag,
    generator_rdseed_mt::core_type core,
    generator_rdseed_mt::index_type index) {
  if (decomposed_subtoken) {
    SPDLOG_LOGGER_TRACE(
        logger_, "[generator->generate_load] worker on core {} enabled", core);
    /* Create contention to the hardware random number generator. */
    while (flag) {
      exot::primitives::seed64();
      exot::primitives::seed64();
      exot::primitives::seed64();
      exot::primitives::seed64();
      exot::primitives::seed64();
    }
  }
}

#endif
