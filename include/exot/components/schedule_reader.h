/**
 * @file components/schedule_reader.h
 * @author     Bruno Klopott
 * @brief      Schedule reader node, outputs user-defined tokens based on file
 *             or console input.
 */

#pragma once

#include <chrono>
#include <fstream>
#include <ios>
#include <iostream>
#include <istream>
#include <optional>
#include <string>
#include <thread>

#include <fmt/format.h>   // for string formatting
#include <fmt/ostream.h>  // for ostream support
#include <spdlog/spdlog.h>

#include <spdlog/sinks/stdout_color_sinks.h>

#include <exot/framework/all.h>            // for framework support
#include <exot/utilities/configuration.h>  // for configurable
#include <exot/utilities/istream.h>        // for reading tuples via >>
#include <exot/utilities/ostream.h>        // for printing tuples
#include <exot/utilities/thread.h>

namespace exot::components {

/**
 * @brief      Class for reading schedule files
 *
 * @tparam     Token  Token type required by the receiving node
 */
template <typename Token>
class schedule_reader : public exot::framework::IProcess,
                        public exot::framework::Producer<Token> {
 public:
  using node_type      = exot::framework::Producer<Token>;
  using token_type     = typename node_type::interface_type::value_type;
  using state_type     = exot::framework::State;
  using state_pointer  = std::shared_ptr<state_type>;
  using logger_pointer = std::shared_ptr<spdlog::logger>;
  using clock_type     = std::chrono::steady_clock;
  using node_type::out_;

  /**
   * @brief      Structure holding module settings
   * @todo REFACTOR with std::optional for `input_file`
   */
  struct settings : public exot::utilities::configurable<settings> {
    std::string input_file;        /*! path to schedule file */
    bool reading_from_file{false}; /*! flag set when reading from file */
    bool read_as_hex{false};       /*! interpret input as hexadecimal? */
    std::optional<unsigned> cpu_to_pin;

    const char* name() const { return "schedule_reader"; }

    /* @brief The JSON configuration function */
    void configure() {
      /* "this->" required for fixing an error that appears only in templated
       * components where declarations in dependent base are not found by
       * unqualified lookup, "'bind_and_describe_data' was not declared in this
       * scope, and no declarations were found by argument-dependent lookup at
       * the point of instantiation" (G++ 8.2.0). */
      this->bind_and_describe_data(
          "input_file", input_file,
          "the input schedule file |string|, e.g. \"input.sched\"");
      this->bind_and_describe_data(
          "reading_from_file", reading_from_file,
          "reading from file? |bool|, reads from stdin if false");
      this->bind_and_describe_data("read_as_hex", read_as_hex,
                                   "read hexadecimal values? |bool|");
      this->bind_and_describe_data("cpu_to_pin", cpu_to_pin,
                                   "schedule reader pinning |uint|, e.g. 7");
    }
  };

  /**
   * @brief      Constructs the schedule reader
   *
   * @param      conf  The settings structure
   */
  schedule_reader(settings& conf) : conf_{conf} {
    if (!conf_.reading_from_file) {
      debug_log_->info("[schedule_reader] reading from standard input");
      input_ = std::make_unique<std::istream>(std::cin.rdbuf());
    } else {
      debug_log_->info("[schedule_reader] reading from file: {}",
                       conf_.input_file);
      input_ = std::make_unique<std::ifstream>(conf_.input_file);
    }

    if (conf_.cpu_to_pin.has_value()) {
      if (conf_.cpu_to_pin.value() >= std::thread::hardware_concurrency()) {
        throw std::logic_error("Supplied wrong CPU to pin the schedule_reader");
      }
    }
  }

  /**
   * @brief      Main process
   */
  void process() override {
    if (conf_.cpu_to_pin.has_value()) {
      exot::utilities::ThreadTraits::set_affinity(conf_.cpu_to_pin.value());
    }

    debug_log_->info("[schedule_reader] running on {}",
                     exot::utilities::thread_info());

    for (std::string line;
         std::getline(*input_, line) && !global_state_->is_stopped();) {
      std::istringstream internal(line);
      token_type current;

      if (conf_.read_as_hex) {
        internal >> std::hex >> current;
      } else {
        internal >> current;
      }

      debug_log_->debug("[schedule_reader] received and formatted tuple: {}",
                        current);
      while (!out_.try_write_for(current, std::chrono::seconds{1}) &&
             !global_state_->is_stopped()) {}
    }

    if (input_->eof()) {
      debug_log_->info(
          "[schedule_reader] reached end of file/stream, stopping global "
          "state");
      global_state_->stop();
    }
  }

 private:
  state_pointer global_state_{exot::framework::GLOBAL_STATE->get()};
  logger_pointer debug_log_ =
      spdlog::get("log") ? spdlog::get("log") : spdlog::stderr_color_mt("log");

  settings conf_;
  std::unique_ptr<std::istream> input_;
};  // namespace exot::components

}  // namespace exot::components
