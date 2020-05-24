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
 * @file utilities/main.h
 * @author     Bruno Klopott
 * @brief      Wrappers for spawning typical applications with or without CLI.
 *
 * @details    The following function templates perform the typical
 *             bootstrapping of applications composed with the framework. The
 *             functions are given framework components, like meter_host_logger,
 *             as template arguments. The components' settings structures
 *             are created and configured with JSON, and passed to the
 *             respective constructors. In the `cli_wrapper` the command line
 *             arguments are also parsed (as given to `main`). Components are
 *             then spawned with the `ThreadExecutor`.
 */

#include <string>
#include <tuple>
#include <type_traits>

#include <nlohmann/json.hpp>

#include <exot/framework/all.h>
#include <exot/utilities/cli.h>
#include <exot/utilities/configuration.h>
#include <exot/utilities/filesystem.h>
#include <exot/utilities/helpers.h>
#include <exot/utilities/logging.h>
#include <exot/utilities/types.h>

namespace exot::utilities {

/**
 * @brief  Provides a cli main wrapper for typical applications
 *
 * @tparam Components  The component types
 * @param  argc        The argc from main
 * @param  argv        The argv from main
 * @return             Return code [0, 1]
 */
template <typename... Components>
auto cli_wrapper(int argc, char** argv) {
  using component_ptrs_t = unique_ptr_tuple<Logging, Components...>;
  using settings_tuple_t =
      std::tuple<Logging::settings, typename Components::settings...>;

  settings_tuple_t settings;
  component_ptrs_t components;

  CLI cli;
  JsonConfig jc;

  cli.add_configurations(jc.get_cli_configuration());

#ifdef __cpp_lib_apply
  std::apply([&](auto&... v) { cli.collect_descriptions(v...); }, settings);
#else
  const_for<0, std::tuple_size_v<settings_tuple_t>>(
      [&](const auto I) { cli.collect_descriptions(std::get<I>(settings)); });
#endif

  if (!cli.parse(argc, argv)) return 1;

#ifdef __cpp_lib_apply
  std::apply([&](auto&... v) { configure(jc, v...); }, settings);
#else
  const_for<0, std::tuple_size_v<settings_tuple_t>>(
      [&](const auto I) { configure(jc, std::get<I>(settings)); });
#endif

  exot::framework::init_global_state_handlers();

  const_for<0, std::tuple_size_v<component_ptrs_t>>([&](const auto I) {
    using component_t =
        typename std::decay_t<decltype(std::get<I>(components))>::element_type;

    std::get<I>(components) =
        std::make_unique<component_t>(std::get<I>(settings));
  });

  if constexpr (sizeof...(Components) > 1ull) {
    auto connector = exot::framework::Connector();

    const_for<1ull, sizeof...(Components)>([&](const auto I) {
      connector.connect(*std::get<I>(components),
                        *std::get<I + 1ull>(components));
    });
  }

  exot::framework::ThreadExecutor executor;

#ifdef __cpp_lib_apply
  std::apply([&](const auto&... v) { executor.spawn((*v)...); },
             tail(std::move(components)));
#else
  const_for<1, std::tuple_size_v<settings_tuple_t>>(
      [&](const auto I) { executor.spawn(*std::get<I>(components)); });
#endif

  executor.join();
  return 0;
}

}  // namespace exot::utilities