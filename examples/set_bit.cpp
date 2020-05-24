/**
 * @file examples/set_bit.cpp
 * @author     Bruno Klopott
 * @brief      An example showing how to loop through set bits of an integer.
 */

#include <fmt/format.h>

#include <exot/utilities/bits.h>

using namespace exot::utilities;

int main(int argc, char** argv) {
  auto value = 0x1020304080F0A01Cull;

  fmt::print(
      "      "
      "6666555555555544444444443333333333222222222211111111110000000000\n");
  fmt::print(
      "      "
      "3210987654321098765432109876543210987654321098765432109876543210\n");
  fmt::print("      {:->64}\n", "");
  fmt::print("{:70b}\n", value);
  fmt::print("      {:->64}\n", "");

  loop_through_set_bits(value, [](std::size_t i) {
    fmt::print("{:2d}{:>{}}\n", i, "x", 68 - i);
  });

  return 0;
}
