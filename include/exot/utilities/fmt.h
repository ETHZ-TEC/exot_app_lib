/**
 * @file utilities/fmt.h
 * @author     Bruno Klopott
 * @brief      String formatting overloads for the fmtlib.
 *
 * @details    Provide fmt-specific template specifications of the contained
 *             ostream-overloads for performance gains.
 */

#pragma once

#include <optional>

#include <fmt/format.h>

/**
 * Overloads that target the fmtlib directly
 */
namespace fmt {

/**
 * @brief      Formatter for optional types
 * @note       Uses default formatter "{}".
 *
 * @tparam     T     The type encapsulated by std::optional
 */
template <typename T>
struct formatter<std::optional<T>> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext& ctx) {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(const std::optional<T>& p, FormatContext& ctx) {
    if (p.has_value()) {
      return format_to(ctx.begin(), "{}", p.value());
    } else {
      return format_to(ctx.begin(), "<nullopt>");
    }
  }
};

}  // namespace fmt
