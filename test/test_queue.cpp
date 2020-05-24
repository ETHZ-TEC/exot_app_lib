#include <limits>

#include <doctest.h>

#include <exot/framework/queue.h>

TEST_SUITE("LockingQueue") {
  TEST_CASE("Default LockingQueue") {
    exot::framework::LockingQueue<int> queue;

    REQUIRE(queue.capacity() == std::numeric_limits<size_t>::max());
    REQUIRE(queue.size() == 0);

    SUBCASE("Empty queue") { CHECK(queue.empty() == true); }
  }
}