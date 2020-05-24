/**
 * @file utilities/cli.h
 * @author     Bruno Klopott
 * @brief      Command line parsing facilities.
 */

#pragma once

#include <fstream>
#include <initializer_list>
#include <sstream>
#include <stdexcept>
#include <string>

#include <clipp.h>
#include <fmt/format.h>
#include <fmt/ostream.h>
#include <nlohmann/json.hpp>

#include <exot/utilities/configuration.h>
#include <exot/utilities/filesystem.h>
#include <exot/utilities/formatting.h>

/**
 * DEBUG_CLI enables more descriptive printing of parsing errors
 */
#ifndef DEBUG
#define DEBUG_CLI false
#else
#define DEBUG_CLI true
#endif

namespace exot::utilities {

class JsonConfig {
 public:
  clipp::group get_cli_configuration();

  nlohmann::json get() const;
  nlohmann::json::reference get_ref();
  nlohmann::json::const_reference get_const_ref() const;
  std::string get_filename_or_data() const;

  void set_with_file(const char* file);
  void set_with_file(const std::string& file);
  void set_with_file(std::string&& file);
  void set_with_string(const char* string);
  void set_with_string(const std::string& string);
  void set_with_string(std::string&& string);

 private:
  void ingest(bool using_a_file);

  std::string json_file_or_data_;
  bool using_a_file_{false};
  nlohmann::json root_json_;
};

/* Template specialisations to make the configuration syntax more compact. */

/**
 * @brief      Sets the JSON file in multiple configurables
 *
 * @param[in]  config     The JsonConfig object
 * @param[in]  ts         The configurables
 *
 * @tparam     Ts         The types of the configurables
 */
template <typename... Ts>
inline void set_json(const JsonConfig& config, Ts&&... ts) {
  (..., ts.set_json(config.get_const_ref()));
}

/**
 * @brief      Sets the JSON file in multiple configurables and configures them
 *
 * @param[in]  config     The JsonConfig object
 * @param[in]  ts         The configurables
 *
 * @tparam     Ts         The types of the configurables
 */
template <typename... Ts>
inline void configure(const JsonConfig& config, Ts&&... ts) {
  set_json(config.get_const_ref(), ts...);
  only_configure(ts...);
}

/**
 * @brief      Class providing the command line interface
 */
class CLI {
 public:
  /**
   * @brief      Adds multiple configurations to the interface
   *
   * @param[in]  config   The individual configuration group
   *
   * @tparam     Configs  Configuration groups
   */
  template <typename... Configs>
  void add_configurations(const Configs&... config) {
    (..., base_.push_front(config));
  }

  /**
   * @brief      Prepends a section to the help message
   *
   * @param[in]  title    The title
   * @param[in]  content  The content
   */
  void prepend_section(const std::string& title, const std::string& content);

  /**
   * @brief      Appends a section to the help message
   *
   * @param[in]  title    The title
   * @param[in]  content  The content
   */
  void append_section(const std::string& title, const std::string& content);

  /**
   * @brief      Adds a description section to the help message
   *
   * @param[in]  description  The description
   */
  void add_description(const std::string& description);

  /**
   * @brief      Adds an example section to the help message
   *
   * @param[in]  example  The example
   */
  void add_example(const std::string& example);

  /**
   * @brief      Parses the command line arguments
   *
   * @param[in]  argc  The argc from main
   * @param      argv  The argv from main
   *
   * @return     True if successful, false if parsing error occurred
   */
  bool parse(int argc, char** argv);

  template <typename... Ts>
  void collect_descriptions(Ts&&... ts) {
    auto descriptions = std::string{};
    (..., descriptions.append(ts.describe()));
    sections_to_append_.emplace_back("CONFIGURATION",
                                     indent(descriptions, 8, false));
  }

 private:
  using section = clipp::man_page::section;

  clipp::group base_{};  //! base configuration group
  std::vector<section> sections_to_prepend_;
  std::vector<section> sections_to_append_;
  const bool debug_{DEBUG_CLI};
};

}  // namespace exot::utilities
