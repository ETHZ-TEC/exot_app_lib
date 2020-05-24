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
 * @file examples/meter.cpp
 * @author     Bruno Klopott
 * @brief      An example meter application, combining all available modules.
 */

/* Step 1: Include necessary headers: */

#define METER_LOG_SYSTEM_TIME false
#include <exot/components/meter_host_logger.h>  // Meter host
#include <exot/framework/all.h>                 // Connector, executor, etc.
#include <exot/meters/frequency.h>              // Frequency modules
#include <exot/meters/power.h>                  // Power modules
#include <exot/meters/thermal.h>                // Thermal modules
#include <exot/meters/utilisation.h>            // Utilisation modules
#include <exot/utilities/cli.h>                 // Command-line parsing
#include <exot/utilities/configuration.h>       // Configuration/JSON
#include <exot/utilities/logging.h>             // Logging facilities

/* Other available meter modules. */
// #include <exot/meters/cache.h>                  // Cache modules
// #include <exot/meters/rdseed.h>                 // RDSEED modules

using namespace exot;

int main(int argc, char** argv) {
  /* Step 2: Declare aliases for convienience. */
  using Meter = components::meter_host_logger<std::chrono::microseconds,    //
                                              modules::utilisation_procfs,  //
                                              modules::frequency_sysfs,     //
                                              modules::frequency_rel,       //
                                              modules::thermal_sysfs,       //
                                              modules::thermal_msr,         //
                                              modules::power_msr>;
  using utilities::Logging;

  /* Step 3: Create objects for command-line parsing and JSON configuration. */
  utilities::CLI cli;
  utilities::JsonConfig jc;

  /* Step 4: Create settings objects for used components. */
  Meter::settings meter_settings;
  Logging::settings logging_settings;

  /* Step 5: Add components or JSON cli-configuration to the CLI parser. */
  cli.add_configurations(jc.get_cli_configuration());

  /* Step 6: Parse the command-line arguments. */
  if (!cli.parse(argc, argv)) return 1;

  /* Step 7: Configure the settings objects with the loaded JSON object. */
  utilities::configure(jc, meter_settings, logging_settings);

  /* Step 8: Initialise global state handlers (INT, USR1). */
  framework::init_global_state_handlers();

  /* Step 9: Create top-level components: logging and the executable. */
  Logging logging(logging_settings);
  Meter meter(meter_settings);

  /* Step 10: Spawn the executable components on an executor and wait. */
  framework::ThreadExecutor exec;
  exec.spawn(meter);
  exec.join();

  return 0;
}
