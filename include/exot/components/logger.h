/**
 * @file components/logger.h
 * @author     Bruno Klopott
 * @brief      Basic logger node, logs any serialisable data structure to
 *             standard output and/or file.
 */

#pragma once

#include <chrono>       // for durations
#include <string>       // for std::string
#include <type_traits>  // for enable_if_t, is_convertible_v

#include <exot/framework/all.h>
#include <exot/utilities/ostream.h>
#include <exot/utilities/timing.h>
#include <exot/utilities/types.h>

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <spdlog/spdlog.h>

#include <spdlog/sinks/stdout_color_sinks.h>

namespace exot::components {

template <typename Token>
class Logger : public exot::framework::Consumer<Token>,
               public exot::framework::IProcess {
 public:
  using node_type = exot::framework::Consumer<Token>;
  using node_type::in_;  //! needed because the base class is a class template

  using clock_type = std::chrono::steady_clock;
  using container_pointer =
      typename node_type::interface_type::container_pointer;
  using input_token_type = typename node_type::interface_type::value_type;
  using state_type       = exot::framework::State;
  using state_pointer    = std::shared_ptr<state_type>;
  using logger_pointer   = std::shared_ptr<spdlog::logger>;

  Logger(){};
  ~Logger() = default;

  /**
   * @brief      Sets the logging header
   *
   * @param[in]  header  The header description
   *
   * @tparam     T          The data type of the header, has to be convertible
   *                        to string or be a tuple, for which an overload is
   *                        available
   * @tparam     <unnamed>  Template helper to enable this template only for
   *                        types that are convertible to std::string or are
   *                        tuples
   */
  template <typename T,
            typename = std::enable_if_t<
                std::is_convertible_v<std::decay_t<T>, std::string> ||
                exot::utilities::is_tuple_v<std::decay_t<T>>>>
  void set_header(T&& header) {
    logging_header_ = fmt::format("{}", header);
  }

  /**
   * @brief      Gets the logging header
   *
   * @return     The header.
   */
  std::string get_header() const { return logging_header_; }

  void process() override {
    using namespace std::chrono_literals;
    input_token_type token;

    if (!logging_header_.empty()) application_log_->info("{}", logging_header_);

    while (!local_state_->is_stopped()) {
      if (in_.try_read_for(token, 100ms)) {
        /* Uses the overloaded ostream `operator<<` to output any tuple or
         * range-like types, separating tokens with a delimeter. */
        application_log_->info("{}", token);
      } else {
        debug_log_->debug("[Logger] reading from input queue timed out");
      }

      if (global_state_->is_stopped() && !in_.is_readable()) {
        debug_log_->info("[Logger] terminated & queue empty");
        local_state_->stop();
      } else if (global_state_->is_stopped() &&
                 global_state_->is_terminated()) {
        debug_log_->info("[Logger] terminated & shutdown");
        local_state_->stop();
      }
    }
  };

 private:
  state_pointer local_state_{std::make_shared<state_type>()};
  std::string logging_header_;

  /**
   * Pointers to debug and application loggers
   */
  logger_pointer debug_log_ =
      spdlog::get("log") ? spdlog::get("log") : spdlog::stderr_color_mt("log");
  logger_pointer application_log_ =
      spdlog::get("app") ? spdlog::get("app") : spdlog::stdout_color_mt("app");
};

}  // namespace exot::components
