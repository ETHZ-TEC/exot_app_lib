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
  using loadgen = exot::components::generator_host<
      std::chrono::microseconds, exot::modules::generator_utilisation_mt>;
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
