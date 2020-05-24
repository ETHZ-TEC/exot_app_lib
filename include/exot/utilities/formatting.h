/**
 * @file utilities/formatting.h
 * @author     Bruno Klopott
 * @brief      Utilities for string manipulation and serialisation.
 */

#pragma once

#include <algorithm>    // for copy
#include <chrono>       // for durations
#include <ratio>        // for ratios
#include <sstream>      // for stringstream
#include <string>       // for std::string
#include <type_traits>  // for enable_if
#include <vector>       // for variable-size arrays

#include <fmt/format.h>  // for formatting strings

#include <exot/utilities/types.h>  // for type traits

namespace exot::utilities {

/**
 * @brief      Splits a string using a delimeter
 *
 * @param[in]  string     The string
 * @param[in]  delimiter  The delimiter
 *
 * @tparam     CharT      Character type (e.g. char, wchar_t)
 *
 * @return     A vector with the tokenized string
 */
template <typename CharT, typename _ = std::enable_if_t<is_character_v<CharT>>>
std::vector<std::basic_string<CharT>> split_string(
    const std::basic_string<CharT>& string, CharT delimiter) {
  using string_type = std::basic_string<CharT>;

  string_type token;
  std::vector<string_type> tokenised;

  std::basic_istringstream<CharT> tstream(string);
  while (std::getline(tstream, token, delimiter)) {
    tokenised.push_back(token);
  }

  return tokenised;
}

/**
 * @brief      Joins a vector of strings into a single delimited string
 *
 * @param[in]  strings    The strings
 * @param[in]  delimiter  The delimiter
 *
 * @tparam     CharT      Character type used by basic_string
 *
 * @return     The joined string
 */
template <typename CharT, typename _ = std::enable_if_t<is_character_v<CharT>>>
std::basic_string<CharT> join_strings(
    const std::vector<std::basic_string<CharT>>& strings, CharT delimiter) {
  std::basic_string<CharT> joined;
  bool first{true};

  for (const auto& string : strings) {
    if (first) {
      joined += string;
      first = false;
    } else {
      joined += delimiter + string;
    }
  }

  return joined;
}

/**
 * @brief      Wraps a string to a desired number of columns
 *
 * @param[in]  to_wrap  String to wrap
 * @param[in]  width    The width of the column in characters
 * @param[in]  indent   The indentation of the next lines
 *
 * @return     The wrapped string
 */
template <typename CharT, typename _ = std::enable_if_t<is_character_v<CharT>>>
std::string wrap(const std::basic_string<CharT>& to_wrap, size_t width,
                 size_t indent = 0) {
  std::basic_istringstream<CharT> in(to_wrap);
  std::basic_ostringstream<CharT> out;
  std::basic_string<CharT> current_word;
  size_t current_position{indent};
  std::basic_string<CharT> indent_string(indent, CharT{' '});

  out << indent_string;

  while (in >> current_word) {
    if (current_position + current_word.size() > width) {
      out << CharT{'\n'} << indent_string;
      current_position = indent;
    }

    out << current_word << ' ';
    current_position += current_word.size() + 1;
  }

  return out.str();
}

template <typename CharT, typename _ = std::enable_if_t<is_character_v<CharT>>>
std::string indent(const std::basic_string<CharT>& to_indent, size_t indent,
                   bool wrap = true) {
  auto lines         = split_string(to_indent, CharT{'\n'});
  auto indent_string = std::basic_string<CharT>(indent, CharT{' '});

  for (auto& line : lines) {
    line =
        wrap ? exot::utilities::wrap(line, 80, indent) : indent_string + line;
  }

  return join_strings(lines, CharT{'\n'});
}

/**
 * @brief      Trim characters at front and back of the string, mostly for
 *             whitespace.
 *
 * @param[in]  to_trim          The string to trim
 * @param[in]  trim_characters  A string with characters to be trimmed
 *
 * @tparam     CharT            Character type use by basic_string
 * @tparam     _                Template helper
 *
 * @return     A trimmed string
 */
template <typename CharT, typename _ = std::enable_if_t<is_character_v<CharT>>>
std::string trim_string(const std::basic_string<CharT>& to_trim,
                        const std::basic_string<CharT>& trim_characters =
                            std::basic_string<CharT>{" \t"}) {
  const auto front = to_trim.find_first_not_of(trim_characters);

  if (front == std::basic_string<CharT>::npos) {
    return std::basic_string<CharT>{""};
  }

  const auto back  = to_trim.find_last_not_of(trim_characters);
  const auto range = back - front + 1;

  return to_trim.substr(front, range);
}

template <typename CharT, typename _ = std::enable_if_t<is_character_v<CharT>>>
std::string trim_string(const std::basic_string<CharT>& to_trim,
                        const CharT* trim_characters) {
  return trim_string(std::forward<decltype(to_trim)>(to_trim),
                     std::basic_string<CharT>{trim_characters});
}

/**
 * @brief      Collapse characters present in a string, preferably whitespace.
 *
 * @param[in]  to_collapse          The string to collapse
 * @param[in]  collapse_characters  A string with characters to
 *
 * @tparam     CharT                The character tupe
 * @tparam     _                    Template helper
 *
 * @return     The collapsed string
 */
template <typename CharT, typename _ = std::enable_if_t<is_character_v<CharT>>>
std::string collapse_string(
    const std::basic_string<CharT>& to_collapse,
    const std::basic_string<CharT>& fill_characters =
        std::basic_string<CharT>{" "},
    const std::basic_string<CharT>& collapse_characters =
        std::basic_string<CharT>{" \t"}) {
  auto output = trim_string(to_collapse, collapse_characters);

  auto front = output.find_first_of(collapse_characters);
  while (front != std::basic_string<CharT>::npos) {
    const auto back  = output.find_first_not_of(collapse_characters, front);
    const auto range = back - front;

    output.replace(front, range, fill_characters);

    front = output.find_first_of(collapse_characters,
                                 front + fill_characters.length());
  }

  return output;
}

template <typename CharT, typename _ = std::enable_if_t<is_character_v<CharT>>>
std::string collapse_string(const std::basic_string<CharT>& to_collapse,
                            const std::basic_string<CharT>& fill_characters,
                            const CharT* collapse_characters) {
  return collapse_string(
      std::forward<decltype(to_collapse)>(to_collapse),
      std::forward<decltype(fill_characters)>(fill_characters),
      std::basic_string<CharT>{collapse_characters});
}

template <typename CharT, typename _ = std::enable_if_t<is_character_v<CharT>>>
std::string collapse_string(const std::basic_string<CharT>& to_collapse,
                            const CharT* fill_characters,
                            const CharT* collapse_characters) {
  return collapse_string(std::forward<decltype(to_collapse)>(to_collapse),
                         std::basic_string<CharT>{fill_characters},
                         std::basic_string<CharT>{collapse_characters});
}

template <typename CharT, typename _ = std::enable_if_t<is_character_v<CharT>>>
std::string collapse_string(
    const std::basic_string<CharT>& to_collapse, const CharT* fill_characters,
    const std::basic_string<CharT>& collapse_characters =
        std::basic_string<CharT>{" \t"}) {
  return collapse_string(
      std::forward<decltype(to_collapse)>(to_collapse),
      std::basic_string<CharT>{fill_characters},
      std::forward<decltype(collapse_characters)>(collapse_characters));
}

/**
 * @brief      Extract the first occurence of a positive number from a string
 *
 * @param[in]  string  The string
 *
 * @return     The extracted number
 */
inline unsigned extract_number(const std::string& string) {
  std::string extracted;

  size_t const f = string.find_first_of("0123456789");
  if (f != std::string::npos) {
    size_t const e = string.find_first_not_of("0123456789", f);
    extracted      = string.substr(f, e != std::string::npos ? e - f : e);
  }

  if (!extracted.empty()) {
    /* A string-to-integer conversion for unsigned numbers seems to exist only
     * for unsigned long data type */
    return static_cast<unsigned>(std::stoul(extracted));
  } else {
    throw std::logic_error(fmt::format(
        "Extraction failed, the string {} does not contain a number.", string));
  }
}

/**
 * @brief      Extract the all occurences of positive numbers from a string
 *
 * @param[in]  in  The string
 *
 * @return     The extracted number
 */
inline std::vector<unsigned> extract_numbers(const std::string& in) {
  size_t f = 0;
  size_t e = 0;
  std::string string{in};
  std::vector<unsigned> numbers;

  while (true) {
    std::string extracted;
    f = string.find_first_of("0123456789");

    if (f != std::string::npos) {
      e         = string.find_first_not_of("0123456789", f);
      extracted = string.substr(f, e != std::string::npos ? e - f : e);

      if (!extracted.empty()) {
        numbers.push_back(static_cast<unsigned>(std::stoul(extracted)));
      } else {
        throw std::logic_error(fmt::format(
            "Extraction failed, the string {} does not contain a number.",
            string));
      }

      if (e == std::string::npos) {
        break;
      } else {
        string = string.substr(e);
      }

    } else {
      break;
    }
  };

  return numbers;
}

/**
 * @brief      Helper function for obtaining an unsigned number from a
 *             hexadecimal string notation, also without the leading '0x'.
 *
 * @param[in]  str   The string
 *
 * @return     The converted value
 */
inline unsigned read_hex_value(const std::string& string) {
  unsigned x;
  std::stringstream ss;
  ss << std::hex << string;
  ss >> x;
  return x;
}

namespace details {
/**
 * A hack to allow constructing ratio strings at compile time
 */
template <char Pre, char Post, char Sep, std::intmax_t Num, std::intmax_t Den>
static const auto ratio_string = fmt::format("{}{}{}{}{}", Pre, Num, Sep, Den,
                                             Post);
}  // namespace details

/**
 * @brief      Get a string description of a ratio
 * @details    The SI unit prefixes of {yocto, zepto, zetta, yotta} cannot yet
 *             be represented in any of the standard built-in types (value
 *             exceeds size limit).
 *
 * @param[in]  ratio  The ratio
 *
 * @tparam     Num    Value of the numerator
 * @tparam     Den    Value of the denominator
 *
 * @return     A string with the SI prefix
 */
template <std::intmax_t Num, std::intmax_t Den>
std::string ratio_to_string(std::ratio<Num, Den> ratio) noexcept {
  /* SI prefixes */
  if constexpr (std::is_same_v<std::decay_t<decltype(ratio)>, std::atto>)
    return "a";
  if constexpr (std::is_same_v<std::decay_t<decltype(ratio)>, std::femto>)
    return "f";
  if constexpr (std::is_same_v<std::decay_t<decltype(ratio)>, std::pico>)
    return "p";
  if constexpr (std::is_same_v<std::decay_t<decltype(ratio)>, std::nano>)
    return "n";
  if constexpr (std::is_same_v<std::decay_t<decltype(ratio)>, std::micro>)
    return "µ";
  if constexpr (std::is_same_v<std::decay_t<decltype(ratio)>, std::milli>)
    return "m";
  if constexpr (std::is_same_v<std::decay_t<decltype(ratio)>, std::centi>)
    return "c";
  if constexpr (std::is_same_v<std::decay_t<decltype(ratio)>, std::deci>)
    return "d";
  if constexpr (std::is_same_v<std::decay_t<decltype(ratio)>, std::deca>)
    return "da";
  if constexpr (std::is_same_v<std::decay_t<decltype(ratio)>, std::hecto>)
    return "h";
  if constexpr (std::is_same_v<std::decay_t<decltype(ratio)>, std::kilo>)
    return "k";
  if constexpr (std::is_same_v<std::decay_t<decltype(ratio)>, std::mega>)
    return "M";
  if constexpr (std::is_same_v<std::decay_t<decltype(ratio)>, std::giga>)
    return "G";
  if constexpr (std::is_same_v<std::decay_t<decltype(ratio)>, std::tera>)
    return "T";
  if constexpr (std::is_same_v<std::decay_t<decltype(ratio)>, std::peta>)
    return "P";
  if constexpr (std::is_same_v<std::decay_t<decltype(ratio)>, std::exa>)
    return "E";

  if constexpr (Num == 1 && Den == 1) return "";

  /* Catch-all */
  return details::ratio_string<'[', ']', '/', Num, Den>;
}

/**
 * @brief      Get a unit prefix of a duration period
 *
 * @param[in]  duration  The duration
 *
 * @tparam     Rep       Type used to store the duration
 * @tparam     Period    The duration period
 *
 * @return     Time unit string with prefixes
 */
template <typename Rep, typename Period>
std::string duration_unit(std::chrono::duration<Rep, Period>) {
  if constexpr (Period::num == 60 && Period::den == 1) return "min";
  if constexpr (Period::num == 3600 && Period::den == 1) return "h";

  return ratio_to_string(Period{}) + "s";
};

/**
 * @brief      Print a duration with its unit
 *
 * @param[in]  duration  The duration
 *
 * @tparam     Rep       Type used to store the duration
 * @tparam     Period    The duration period
 *
 * @return     String with the duration and its unit with prefixes
 */
template <typename Rep, typename Period>
std::string duration_to_string(std::chrono::duration<Rep, Period> duration) {
  return std::to_string(duration.count()) + duration_unit(duration);
}

namespace details {
/**
 * @brief      Prefix, postfix, and delimeter used in serialising ranges.
 * @details    Characters can be chosen individually for different types,
 * using the type trait functions from <type_traits> or "types.h".
 *
 * @tparam     CharT   Character type (e.g. char, wchar_t)
 * @tparam     T       Range type
 * @tparam     Enable  Used for `std::enable_if`-based polymorphism
 */
template <typename CharT, class T, typename Enable = void>
struct Separators {
  const CharT prefix;
  const CharT postfix;
  const CharT delimeter;

  constexpr Separators() : prefix{'\0'}, postfix{'\0'}, delimeter{','} {};
};

template <typename CharT, class T>
struct Separators<CharT, T,
                  typename std::enable_if_t<is_iterable_v<T>>::value> {
  const CharT prefix;
  const CharT postfix;
  const CharT delimeter;

  constexpr Separators() : prefix{'\0'}, postfix{'\0'}, delimeter{','} {};
};

template <typename CharT, class T>
struct Separators<CharT, T,
                  typename std::enable_if_t<is_const_iterable_v<T>>::value> {
  const CharT prefix;
  const CharT postfix;
  const CharT delimeter;

  constexpr Separators() : prefix{'\0'}, postfix{'\0'}, delimeter{','} {};
};

template <typename CharT, class T>
struct Separators<CharT, T, typename std::enable_if_t<is_tuple_v<T>>::value> {
  const CharT prefix;
  const CharT postfix;
  const CharT delimeter;

  constexpr Separators() : prefix{'\0'}, postfix{'\0'}, delimeter{','} {};
};

}  // namespace details

}  // namespace exot::utilities
