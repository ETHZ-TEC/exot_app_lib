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
#include <doctest.h>

#include <exot/utilities/configuration.h>

#include <algorithm>
#include <numeric>
#include <random>
#include <thread>
#include <vector>

using namespace exot::utilities;

TEST_SUITE("Configuration") {
  std::random_device rd;
  std::mt19937 rdg(rd());

  TEST_CASE("Bootstrapping CPU core vectors") {
    using vector = std::vector<unsigned>;

    static unsigned core_count = std::thread::hardware_concurrency();

    REQUIRE(core_count >= 1);

    vector def;
    def.resize(core_count);
    std::iota(def.begin(), def.end(), 0);

    REQUIRE(def.size() == core_count);

    SUBCASE("Bootstrapping given an empty vector") {
      vector empty;

      bootstrap_cores(empty);

      CHECK(std::is_sorted(empty.begin(), empty.end()));
      CHECK(empty == def);
    }

    SUBCASE("Bootstrapping given an unsorted vector") {
      vector provided = def;

      while (provided == def) {
        std::shuffle(provided.begin(), provided.end(), rdg);
      }

      REQUIRE(provided != def);

      bootstrap_cores(provided);

      CHECK(std::is_sorted(provided.begin(), provided.end()));
      CHECK(provided == def);
    }

    SUBCASE("Bootstrapping given a vector with duplicates") {
      vector duplicated = {0, 0, 1, 1, 0, 0, 1, 1};
      vector required   = {0, 1};

      bootstrap_cores(duplicated);

      CHECK(std::is_sorted(duplicated.begin(), duplicated.end()));
      CHECK(duplicated == required);
    }

    SUBCASE("Bootstrapping given values exceeding core count") {
      vector exceeding{core_count + 1, core_count + 2};

      CHECK_THROWS(bootstrap_cores(exceeding));
    }
  }
}
