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
#include <array>
#include <iostream>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

template <typename CharT, typename CharTraitsT, class Current, class... Rest>
std::tuple<Current, Rest...> read_tuple(
    std::basic_istream<CharT, CharTraitsT>& stream) {
  Current current;
  stream >> current;

  if constexpr (sizeof...(Rest) == 0) {
    return std::tuple{current};
  } else {
    return std::tuple_cat(std::tuple{current},
                          read_tuple<CharT, CharTraitsT, Rest...>(stream));
  }
}

namespace details {

template <typename CharT, typename CharTraitsT>
inline auto read_into_stringstream(
    std::basic_istream<CharT, CharTraitsT>& istream)
    -> std::basic_istringstream<CharT> {
  std::basic_string<CharT> token;
  istream >> token;
  return std::move(std::basic_istringstream<CharT>{token});
}

}  // namespace details

namespace std {

template <typename T, typename CharT, typename CharTraitsT>
std::basic_istream<CharT, CharTraitsT>& operator>>(
    std::basic_istream<CharT, CharTraitsT>& istream,
    std::vector<T>& container) {
  using value_type  = typename std::vector<T>::value_type;
  using string_type = std::basic_string<CharT>;
  using stream_type = std::basic_istringstream<CharT>;

  static constexpr auto delimiter = CharT{','};

  auto stream = details::read_into_stringstream(istream);
  for (auto [token, input] = std::make_tuple(value_type{}, string_type{});  //
       std::getline(stream, input, delimiter);) {
    stream_type{input} >> token;
    container.push_back(token);
  }

  return istream;
}

template <typename T, size_t N, typename CharT, typename CharTraitsT>
std::basic_istream<CharT, CharTraitsT>& operator>>(
    std::basic_istream<CharT, CharTraitsT>& istream,
    std::array<T, N>& container) {
  using string_type = std::basic_string<CharT>;
  using stream_type = std::basic_istringstream<CharT>;

  static constexpr auto delimiter = CharT{','};

  auto stream = details::read_into_stringstream(istream);
  for (auto [i, input] = std::make_tuple(size_t{0}, string_type{});
       std::getline(stream, input, delimiter) && i < N; ++i) {
    stream_type{input} >> container.at(i);
  }

  return istream;
};

template <typename CharT, typename CharTraitsT, class... Ts>
std::basic_istream<CharT, CharTraitsT>& operator>>(
    std::basic_istream<CharT, CharTraitsT>& stream, std::tuple<Ts...>& tuple) {
  tuple = read_tuple<CharT, CharTraitsT, Ts...>(stream);
  return stream;
}

}  // namespace std

namespace std {

template <typename T, typename CharT, typename CharTraitsT>
std::basic_ostream<CharT, CharTraitsT>& operator<<(
    std::basic_ostream<CharT, CharTraitsT>& ostream,
    const std::vector<T>& range) {
  ostream << CharT{'['};
  for (typename std::vector<T>::const_iterator it = range.begin();
       it != range.end(); ++it) {
    if (it == range.begin()) {
      ostream << *it;
    } else {
      ostream << CharT{','} << *it;
    }
  }
  ostream << CharT{']'};
  return ostream;
}

template <typename T, size_t N, typename CharT, typename CharTraitsT>
std::basic_ostream<CharT, CharTraitsT>& operator<<(
    std::basic_ostream<CharT, CharTraitsT>& ostream,
    const std::array<T, N>& range) {
  ostream << CharT{'['};
  for (size_t i = 0ull; i < N; ++i) {
    if (i > 0ull) {
      ostream << CharT{','} << range.at(i);
    } else {
      ostream << range.at(i);
    }
  }
  ostream << CharT{']'};
  return ostream;
}

}  // namespace std

// using token_type = std::tuple<int, std::vector<int>, std::array<double, 3>>;
using token_type =
    std::tuple<int, std::vector<int>, std::array<double, 3>, int>;

int main(int argc, char* argv[]) {
  token_type v;

  std::cin >> v;

  std::cout << "0: " << std::get<0>(v) << "\n";
  std::cout << "1: " << std::get<1>(v) << "\n";
  std::cout << "2: " << std::get<2>(v) << "\n";
  std::cout << "3: " << std::get<3>(v) << "\n";

  return 0;
}
