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
      vector required = {0, 1};

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
