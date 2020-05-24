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
 * @file examples/generator.cpp
 * @author     Bruno Klopott
 * @brief      An example generator application, producing synthetic load.
 */

#include <exot/components/generator_host.h>
#include <exot/components/schedule_reader.h>
#include <exot/framework/all.h>
#include <exot/generators/utilisation_mt.h>
#include <exot/utilities/cli.h>
#include <exot/utilities/configuration.h>
#include <exot/utilities/logging.h>

int main(int argc, char** argv) {
  using loadgen =
      exot::components::generator_host<std::chrono::microseconds,
                                       exot::modules::generator_utilisation_mt>;
  using reader =
      exot::components::schedule_reader<typename loadgen::token_type>;
  using logger = exot::utilities::Logging;
  using exot::utilities::CLI;
  using exot::utilities::JsonConfig;

  CLI cli;
  JsonConfig jc;

  cli.add_description(
      "This multithreaded generator can be used to generate synthetic load on
      a specified number of cores.");

  cli.add_configurations(jc.get_cli_configuration());

  if (!cli.parse(argc, argv)) return 1;

  loadgen::settings loadgen_settings;
  reader::settings reader_settings;
  logger::settings logger_settings;

  exot::utilities::configure(jc.get_const_ref(), loadgen_settings,
                               reader_settings, logger_settings);

  exot::framework::init_global_state_handlers();

  logger log(logger_settings);
  reader read(reader_settings);
  loadgen load(loadgen_settings);

  exot::framework::Connector().pipeline(read, load);

  exot::framework::ThreadExecutor exec;
  exec.spawn(read, load);
  exec.join();

  return 0;
}
