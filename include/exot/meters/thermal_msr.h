/**
 * @file meters/thermal_msr.h
 * @author     Bruno Klopott
 * @brief      Thermal metering module utilising Intel's MSRs.
 */

#pragma once

#if defined(__linux__) && !defined(__ANDROID__) && defined(__x86_64__)

#include <cstdint>  // for std::uint*
#include <string>   // for std::string
#include <vector>   // for variable-size arrays

#include <spdlog/spdlog.h>

#include <spdlog/sinks/stdout_color_sinks.h>

#include <exot/meters/base.h>
#include <exot/primitives/msr.h>           // for MSR
#include <exot/utilities/configuration.h>  // for configurable

namespace exot::modules {

/**
 * @brief      Module to read temperatures via model specific registers on Intel
 *             processors
 * @details    The module supports reading core temperatures, and package
 *             temperatures, as long as the processor supports it. Support for
 *             package-level thermal information is checked upon instantiation.
 */
struct thermal_msr : module {
  using return_type = std::vector<unsigned short>;

  /**
   * @brief      Settings supported by the module
   */
  struct settings : public exot::utilities::configurable<settings> {
    std::vector<unsigned> cores;  //! vector of cores
    bool use_package{false};      //! flag to access package temperatures

    const char* name() const { return "thermal_msr"; }

    void configure() {
      bind_and_describe_data(
          "cores", cores, "cores to read temperature of |uint[]|, e.g. [0, 2]");
      bind_and_describe_data("package", use_package,
                             "access the package temperatures? |bool|");
    }
  };

  /**
   * @brief      Construct the MSR thermal module
   *
   * @param      conf  Module settings
   */
  explicit thermal_msr(settings& conf);

  /**
   * @brief      Main measurement function
   *
   * @return     A vector of desired temperatures
   */
  return_type measure();

  /**
   * @brief      Get descriptions of return values
   * @details    The descriptions contain variable names and units
   *
   * @return     A vector with descriptions
   */
  std::vector<std::string> header();

 private:
  using logger_pointer = std::shared_ptr<spdlog::logger>;
  logger_pointer debug_log_ =
      spdlog::get("log") ? spdlog::get("log") : spdlog::stderr_color_mt("log");

  settings conf_;                        //! local settings
  exot::primitives::MSR msr_;            //! the MSR access component
  std::uint64_t temperature_target_;     //! the reference temperature
  std::vector<std::uint64_t> readings_;  //! vector holding temperature readings

  /**
   * @brief      Provide sensible settings
   * @details    The used MSR components will throw when any of the selected
   *             cores exceed hardware concurrency, so there is no need to
   *             perform that check here.
   *
   * @param      conf  Module settings
   *
   * @return     Module settings
   */
  settings validate_settings(settings& conf);

  /**
   * @brief      Convienience wrapper for getting the temperature reading
   *
   * @param[in]  raw   The raw MSR value
   *
   * @return     The temperature.
   */
  unsigned short get_temperature(std::uint64_t raw);

  /**
   * @brief      Convienience wrapper for getting temperature readings
   *
   * @param[in]  raw  The raw MSR values
   *
   * @return     The temperatures.
   */
  return_type get_temperatures(std::vector<std::uint64_t>&& raw);

  /**
   * Relevant MSR register addresses
   */
  const std::uint64_t IA32_TEMPERATURE_TARGET   = 0x1A2;
  const std::uint64_t IA32_THERM_STATUS         = 0x19C;
  const std::uint64_t IA32_PACKAGE_THERM_STATUS = 0x1B1;
};

}  // namespace exot::modules

#endif
