#include <doctest.h>
#include <fmt/format.h>
#include <fmt/ostream.h>
#include <spdlog/spdlog.h>

#include <exot/utilities/logging.h>

using namespace exot::utilities;

TEST_SUITE("Logging") {

  TEST_CASE("Construction of logs") {
    Logging::settings settings;
    settings.async = true;
    settings.log_level = spdlog::level::err;

    Logging logging(settings);

    REQUIRE(spdlog::get("log") != nullptr);
  }
}
