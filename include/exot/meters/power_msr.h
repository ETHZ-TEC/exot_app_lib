/**
 * @file meters/power_msr.h
 * @author     Bruno Klopott
 * @brief      Module for measuring power via energy values accessed through
 *             Intel's MSRs.
 */

#pragma once

#if defined(__linux__) && defined(__x86_64__)

#include <chrono>  // for durations and time points
#include <map>     // for std::map
#include <string>  // for std::string
#include <thread>  // for sleep_for
#include <vector>  // for variable-size arrays

#include <fmt/format.h>  // for formatting strings
#include <spdlog/spdlog.h>

#include <spdlog/sinks/stdout_color_sinks.h>

#include <exot/meters/base.h>
#include <exot/primitives/msr.h>           // for MSR
#include <exot/utilities/bits.h>           // for bit manipulation functions
#include <exot/utilities/configuration.h>  // for configurable

namespace exot::modules {

struct power_msr : module {
  using return_type = std::vector<double>;

  enum class rapl_domain : unsigned short { pp0 = 0, pp1, pkg, sys };

  /**
   * @brief      Settings supported by the module
   */
  struct settings : public exot::utilities::configurable<settings> {
    std::vector<rapl_domain> domains;

    const char* name() const { return "power_msr"; }

    /* @brief The JSON configuration function */
    void configure() {
      bind_and_describe_data(
          "rapl_domains", domains,
          "list of RAPL domains to use |rapl_domain[]|, choose "
          "from [\"pp0\", \"pp1\", \"pkg\", \"sys\"]");
    }
  };

  /**
   * @brief      Construct the MSR power module
   *
   * @param      conf  Module settings
   */
  explicit power_msr(settings& conf);

  /**
   * @brief      Main measurement function
   *
   * @return     A vector of
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

  settings conf_;              //! local settings
  exot::primitives::MSR msr_;  //! the MSR access component

  /**
   * Variables used for performing measurements
   */
  double energy_status_unit_; /*! multiplier defining units in which the
                               * processor communicates energy readings */
  std::chrono::time_point<std::chrono::steady_clock>
      previous_time_point_; /*! time points need to be stored to get elapsed
                             * time */
  return_type output_;      /*! vector holding computed output values */
  std::map<rapl_domain, std::uint64_t>
      readings_; /*! map for holding new raw energy readings */
  std::map<rapl_domain, std::uint64_t>
      old_readings_; /*! map for holding old raw energy readings */

  /**
   * @brief      Compute power from energy readings
   *
   * @param[in]  new_energy    The new energy
   * @param[in]  old_energy    The old energy
   * @param[in]  elapsed_time  The elapsed time
   *
   * @return     Average power over the interval in Watts
   */
  inline double energy_to_power(std::uint64_t new_energy,
                                std::uint64_t old_energy, double elapsed_time) {
    auto overflow = (new_energy < old_energy) ? (1ULL << 32) : 0ULL;
    return (static_cast<double>((new_energy + overflow) - old_energy) *
            energy_status_unit_ / elapsed_time);
  }

  /**
   * @brief      Wrapper for reading energy values via MSR
   *
   * @param[in]  domain  The power domain to read
   *
   * @return     The energy reading, in processor-specific units
   */
  inline std::uint64_t get_energy(rapl_domain domain) {
    return exot::utilities::extract_bit_range(
        msr_.read_first(domain_to_msr_[domain]), 0u, 31u);
  }

  std::string domain_to_string(rapl_domain d);

  /**
   * Relevant MSR register addresses
   */
  const std::uint64_t PP0_ENERGY_STATUS{0x639};
  const std::uint64_t PP1_ENERGY_STATUS{0x641};
  const std::uint64_t PKG_ENERGY_STATUS{0x611};
  const std::uint64_t PLATFORM_ENERGY_STATUS{0x64d};
  const std::uint64_t RAPL_POWER_UNIT{0x606};

  /**
   * Map between domains and relevant registers
   */
  std::map<rapl_domain, std::uint64_t> domain_to_msr_{
      {rapl_domain::pp0, PP0_ENERGY_STATUS},
      {rapl_domain::pp1, PP1_ENERGY_STATUS},
      {rapl_domain::pkg, PKG_ENERGY_STATUS},
      {rapl_domain::sys, PLATFORM_ENERGY_STATUS}};
};

/* @note       The following overload will also work with arrays of data. */

/**
 * @brief      Overload for rapl_domain enumeration.
 *
 * @param[in]  j     The json object
 * @param      d     The enum object to write data to
 */
inline void from_json(const nlohmann::json& j, power_msr::rapl_domain& d) {
  if (!j.is_string()) {
    throw nlohmann::json::type_error::create(
        302, fmt::format("type must be a string, but was a {}", j.type_name()));
  } else {
    auto value = j.get<nlohmann::json::string_t>();
    if (value == "pp0") {
      d = power_msr::rapl_domain::pp0;
    } else if (value == "pp1") {
      d = power_msr::rapl_domain::pp1;
    } else if (value == "pkg") {
      d = power_msr::rapl_domain::pkg;
    } else if (value == "sys") {
      d = power_msr::rapl_domain::sys;
    } else {
      throw nlohmann::json::other_error::create(
          501, fmt::format("provided rapl domain value \"{}\" is not "
                           "featured in the enum class",
                           value));
    }
  }
}

/**
 * @brief      Overload for serialising rapl_domain enumeration.
 *
 * @param[in]  j     The json object
 * @param      d     The enum object to serialise
 */
inline void to_json(nlohmann::json& j, const power_msr::rapl_domain& d) {
  switch (d) {
    case power_msr::rapl_domain::pp0:
      j = "pp0";
      break;
    case power_msr::rapl_domain::pp1:
      j = "pp1";
      break;
    case power_msr::rapl_domain::pkg:
      j = "pkg";
      break;
    case power_msr::rapl_domain::sys:
      j = "sys";
      break;
  }
}

}  // namespace exot::modules

#endif
