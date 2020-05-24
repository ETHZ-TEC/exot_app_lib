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
 * @file utilities/configuration.cpp
 * @author     Bruno Klopott
 * @brief      Implementation of the bootstrapping utilities from
 *             @ref configuration.h.
 */

#include <exot/utilities/configuration.h>

namespace exot::utilities {

void bootstrap_cores(std::vector<unsigned>& cores) {
  auto concurrency = std::thread::hardware_concurrency();

  if (cores.empty()) {
    cores.resize(concurrency);
    std::iota(cores.begin(), cores.end(), 0);
  }

  /* Make sure that no provided core exceeds available core number.
   */
  if (std::any_of(cores.begin(), cores.end(),
                  [=](auto el) { return el >= concurrency; })) {
    throw std::out_of_range(
        "At least one element in the CPU vector exceeds hardware "
        "concurrency.");
  }

  /* Sort and remove duplicates from the vector of cores
   */
  sort_and_deduplicate(cores);
}

}  // namespace exot::utilities
