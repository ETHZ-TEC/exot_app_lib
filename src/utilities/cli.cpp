/**
 * @file utilities/cli.cpp
 * @author     Bruno Klopott
 * @brief      Implementation of the command line parser class from @ref cli.h.
 */

#include <exot/utilities/cli.h>

namespace exot::utilities {

clipp::group JsonConfig::get_cli_configuration() {
  using clipp::group;
  using clipp::option;
  using clipp::value;

  auto file   = (option("--json_file").required(true) &
               value("json file")
                   .doc("Configure with a file")
                   .call([&](const char* value) { set_with_file(value); }));
  auto string = (option("--json_string").required(true) &
                 value("json string")
                     .doc("Configure by passing a string")
                     .call([&](const char* value) { set_with_string(value); }));

  auto config = (file | string);
  config.doc("JSON-based configuration");

  return config;
}

nlohmann::json JsonConfig::get() const {
  return root_json_;
}

nlohmann::json::reference JsonConfig::get_ref() {
  return root_json_;
}

nlohmann::json::const_reference JsonConfig::get_const_ref() const {
  return root_json_;
}

std::string JsonConfig::get_filename_or_data() const {
  return json_file_or_data_;
}

void JsonConfig::set_with_file(const char* file) {
  json_file_or_data_ = std::string{file};
  ingest(true);
}

void JsonConfig::set_with_file(const std::string& file) {
  json_file_or_data_ = file;
  ingest(true);
}

void JsonConfig::set_with_file(std::string&& file) {
  json_file_or_data_ = std::move(file);
  ingest(true);
}

void JsonConfig::set_with_string(const char* string) {
  json_file_or_data_ = std::string{string};
  ingest(false);
}

void JsonConfig::set_with_string(const std::string& string) {
  json_file_or_data_ = string;
  ingest(false);
}

void JsonConfig::set_with_string(std::string&& string) {
  json_file_or_data_ = std::move(string);
  ingest(false);
}

void JsonConfig::ingest(bool using_a_file) {
  if (using_a_file) {
    if (!is_readable(json_file_or_data_)) {
      throw std::logic_error(
          fmt::format("The supplied JSON-file \"{}\" is not readable.",
                      json_file_or_data_));
    }

    std::ifstream file{json_file_or_data_};
    file >> root_json_;  // may throw JSON-specific exceptions
  } else {
    root_json_ = nlohmann::json::parse(json_file_or_data_);
  }
}

void CLI::prepend_section(const std::string& title,
                          const std::string& content) {
  sections_to_prepend_.emplace(sections_to_prepend_.begin(),  //
                               title, wrap(content, 80, 8));
}

void CLI::append_section(const std::string& title, const std::string& content) {
  sections_to_append_.emplace_back(title, wrap(content, 80, 8));
}

void CLI::add_description(const std::string& description) {
  prepend_section("DESCRIPTION", description);
}

void CLI::add_example(const std::string& example) {
  append_section("EXAMPLE", example);
}

bool CLI::parse(int argc, char** argv) {
  using clipp::make_man_page;
  using clipp::option;

  bool help{false};

  auto cli = clipp::group{};

  if (!base_.empty()) {
    cli = clipp::group(
        base_ | (option("-h", "--help").set(help)).doc("print help message"));
  } else {
    cli = clipp::group(option("-h", "--help").set(help))
              .doc("print help message");
  }

  auto cli_result = clipp::parse(argc, argv, cli);
  auto man_page   = make_man_page(cli, argv[0]);

  if (!sections_to_prepend_.empty()) {
    for (const auto& section : sections_to_prepend_) {
      man_page.prepend_section(section.title(), section.content());
    }
  }

  if (!sections_to_append_.empty()) {
    for (const auto& section : sections_to_append_) {
      man_page.append_section(section.title(), section.content());
    }
  }

  int missing{0};
  int blocked{0};
  int conflicted{0};

  for (const auto& miss : cli_result.missing()) {
    if (debug_) {
      std::stringstream param;
      param << miss.param();
      fmt::print(stderr, "missing: {}, {}\n", param.str(), miss.after_index());
    }
    ++missing;
  }

  for (const auto& match : cli_result) {
    if (debug_) {
      std::stringstream param;
      param << match.param();
      fmt::print(stderr, "match: {}, arg: {}, param: {}, repeat: {}",  //
                 match.index(), match.arg(), param.str(), match.repeat());
    }

    if (match.blocked()) {
      if (debug_) fmt::print(stderr, ", blocked");
      ++blocked;
    }

    if (match.conflict()) {
      if (debug_) fmt::print(stderr, ", conflict");
      ++conflicted;
    }

    if (debug_) fmt::print(stderr, "\n");
  }

  if ((missing > 0 || conflicted > 0) || help) {
    if (!help) {
      fmt::print(stderr,
                 "Parsing error: {} missing/incorrect, "
                 "{} blocked, {} conflicts\n\n",  //
                 missing, blocked, conflicted);
    }

    fmt::print(stderr, "{}", man_page);
    return false;
  }

  return true;
}

}  // namespace exot::utilities
