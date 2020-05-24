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
 * @file components/meter_host.h
 * @author     Bruno Klopott
 * @brief      Node combining multiple metering modules together into a single
 *             process network node.
 * @warning    Current development is only done on the 'meter_host_logger'
 *             component, therefore the following code might not be the most
 *             recent or have equal level of support.
 */

#pragma once

#include <chrono>
#include <string>
#include <thread>

#include <fmt/format.h>  // for string formatting
#include <spdlog/spdlog.h>

#include <spdlog/sinks/stdout_color_sinks.h>

#include <exot/framework/all.h>  // for framework support
#include <exot/meters/base.h>
#include <exot/utilities/configuration.h>
#include <exot/utilities/formatting.h>
#include <exot/utilities/thread.h>
#include <exot/utilities/timing.h>

#include <exot/utilities/ostream.h>
#include <fmt/ostream.h>

namespace exot::components {

namespace details {
template <typename Duration, typename... Meters>
using meter_token_type = std::tuple<Duration, typename Meters::return_type...>;
}

/**
 * @brief      The process network component hosting meter modules
 *
 * @tparam     Duration  The duration type
 * @tparam     Meters    The meter modules
 */
template <typename Duration, typename... Meters>
class meter_host : public Meters...,
                   public exot::framework::IProcess,
                   public exot::framework::Producer<
                       details::meter_token_type<Duration, Meters...>> {
 public:
  using node_type =
      exot::framework::Producer<details::meter_token_type<Duration, Meters...>>;
  using token_type     = typename node_type::interface_type::value_type;
  using state_type     = exot::framework::State;
  using state_pointer  = std::shared_ptr<state_type>;
  using logger_pointer = std::shared_ptr<spdlog::logger>;
  using clock_type     = std::chrono::steady_clock;
  using policy_type    = exot::utilities::SchedulingPolicy;
  using node_type::out_;

  /**
   * Flag set at compile time for logging times and providing timing statistics.
   */
  static constexpr bool use_statistics = false;

  /**
   * @brief      The settings structure, inherits from individual modules
   * settings.
   */
  struct settings : public exot::utilities::configurable<settings>,
                    Meters::settings... {
    Duration period{std::chrono::milliseconds{10}};

    /// @defgroup meter_host_settings Meter host's master thread settings
    /// @{
    unsigned host_pinning{0};                     //! core to pin host to
    bool should_pin_host{false};                  //! should pin host?
    unsigned host_priority{90};                   //! host's priority
    policy_type host_policy{policy_type::Other};  //! host's sched. policy
    /// @}

    using base_t = exot::utilities::configurable<settings>;

    const char* name() const { return "meter"; }

    /* @brief The combining settings structure needs to overload this to allow
     * initialising JSON configs in inherited classes. */
    void set_json(const nlohmann::json& root) {
      json_config_ = base_t::set_json(root);

      /* Fold expression over all meter module configure functions. */
      (..., Meters::settings::set_json(root));
    }

    std::string describe() {
      auto description = base_t::describe();
      (..., description.append(Meters::settings::describe()));
      return description;
    }

    /* @brief The JSON configuration function */
    void configure() {
      base_t::bind_and_describe_data("period", period,
                                     "sampling period |s|, e.g. 1.0e-3");
      base_t::bind_and_describe_data(
          "host_priority", host_priority,
          "host's scheduling priority |uint|, in range [0, 99], e.g. 99");
      base_t::bind_and_describe_data(
          "host_policy", host_policy,
          "host's scheduling policy |str, policy_type|, e.g. \"round_robin\"");

      static_assert(
          (exot::utilities::is_configurable_v<Meters::settings> && ...),
          "Meters must satisfy is_configurable to be used with JSON "
          "configuration.");

      /* Fold expression over all meter module configure functions. */
      (..., Meters::settings::configure());
    }
  };

  static_assert((exot::modules::has_meter_function<Meters>::value && ...),
                "Mixins need to provide a measure() function.");

  /**
   * @brief      Constructs the meter host
   *
   * @param      conf  The settings object
   */
  meter_host(settings& conf) : Meters(conf)..., conf_{conf} {
    debug_log_->info("[meter] using period: {}",
                     exot::utilities::duration_to_string(conf_.period));
    if (conf_.should_pin_host) {
      debug_log_->info("[meter] will run pinned to {}", conf_.host_pinning);
    } else {
      debug_log_->info("[meter] will run not pinned");
    }
  }

  /**
   * @brief      The main process
   */
  void process() override {
    auto until  = [this]() { return !global_state_->is_stopped(); };
    auto action = [this]() { out_.write(measure()); };

    if (conf_.should_pin_host)
      exot::utilities::ThreadTraits::set_affinity(conf_.host_pinning);

    exot::utilities::ThreadTraits::set_scheduling(conf_.host_policy,
                                                  conf_.host_priority);

    debug_log_->info("[meter] running on {}", exot::utilities::thread_info());

    while (!global_state_->is_started()) {
      std::this_thread::sleep_for(std::chrono::milliseconds{100});
    }

    /* Run `action` till `until` returns false. */
    timer_.run_every(conf_.period, until, action);
  }

  /**
   * @brief      Perform measurement
   *
   * @return     Timestamped results of all individual meters
   */
  inline token_type measure() {
    return std::make_tuple(
        std::chrono::duration_cast<Duration>(
            std::chrono::steady_clock::now().time_since_epoch()),
        std::move(Meters::measure())...);
  }

  auto variable_names() {
    return std::make_tuple(timestamp_name(), Meters::variable_names()...);
  }
  auto variable_units() {
    return std::make_tuple(timestamp_unit(), Meters::variable_units()...);
  }
  auto header() {
    return std::make_tuple(timestamp_name_and_unit(), Meters::header()...);
  }

  ~meter_host() {
    if constexpr (use_statistics) {
      debug_log_->info("[meter] timing offset statistics: {}",
                       timer_.offset_statistics());
      debug_log_->info("[meter] timing interval statistics: {}",
                       timer_.interval_statistics());
    }
  }

 private:
  state_pointer global_state_{exot::framework::GLOBAL_STATE->get()};
  logger_pointer debug_log_ =
      spdlog::get("log") ? spdlog::get("log") : spdlog::stderr_color_mt("log");
  exot::utilities::TimeKeeper<clock_type, use_statistics,
                              std::chrono::microseconds>
      timer_;

  settings conf_;

  std::string timestamp_name() { return "timestamp"; }
  std::string timestamp_unit() {
    return exot::utilities::duration_unit(conf_.period);
  }
  std::string timestamp_name_and_unit() {
    return fmt::format("{}_[{}]", timestamp_name(), timestamp_unit());
  }
};

//
}  // namespace exot::components
