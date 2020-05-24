/**
 * @file examples/configurable.cpp
 * @author     Bruno Klopott
 * @brief      An example configurable type.
 */

#include <map>
#include <string>
#include <type_traits>
#include <vector>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <nlohmann/json.hpp>

#include <exot/utilities/configuration.h>

/* Example 1: How to make a custom type work with JSON? */

/* A simple non-template type. */
struct MySimpleT {
  int value_;
};

/* A custom class template that wraps an arithmentic type or nullptr.
 * In JSON, it will be represented as: {"value": ..., "description": ...}. */
template <typename WrappedType,
          typename = std::enable_if_t<(!std::is_null_pointer_v<WrappedType> &&
                                       std::is_arithmetic_v<WrappedType>)>>
struct MyTemplateT {
  WrappedType value_;        //! A value of type WrappedType
  std::string description_;  //! A description of type std::string
};

/* To create a custom JSON serialiser, it's easiest to provide a template
 * specialisation for the adl_serialiser. The specialisation has to be created
 * in the nlohmann namespace. */

/* To allow a type T to be serialised to JSON, create a to_json function, with
 * signature to_json(json&, const T&). */

/* To allow a type T to be created from JSON, create a from_json function, with
 * signature from_json(const json&, T&). */

namespace nlohmann {
/* Simple types: specialise adl_serializer with your type, here: MySimpleT. */
template <>
struct adl_serializer<MySimpleT> {
  static void to_json(json& j, const MySimpleT& o) { j = o.value_; }

  static void from_json(const json& j, MySimpleT& o) { j.get_to(o.value_); }
};

/* Template types: provide a partial specialisation with your type. */
template <typename T>
struct adl_serializer<MyTemplateT<T>> {
  static void to_json(json& j, const MyTemplateT<T>& o) {
    j = json({"value", o.value_}, {"description", o.description_});
  }

  static void from_json(const json& j, MyTemplateT<T>& o) {
    j.at("value").get_to(o.value_);
    j.at("description").get_to(o.description_);
  }
};
}  // namespace nlohmann

using namespace exot::utilities;
using nlohmann::json;

/* Example 2: How to create a configurable class? */

/* First, to make a class 'configurable', you need to inherit from it in a
 * CRTP fashion, i.e. inherit from a class which has the derived class as a
 * template parameter. Thanks to this pattern, the class MyConfigurableClass
 * is accessible inside configurable<...>.
 *
 * Extra care might be needed when your class is itself a class template. See
 * the declaration of settings inside meter_host and meter_host_logger. */
struct MyConfigurableClass : configurable<MyConfigurableClass> {
  /* Declare member variables that you wish to configure. These can be quite
   * complex types (arrays, maps, etc.), or custom types, as long as a
   * serialiser is provided.
   *
   * Below we have the following types of variables: */
  std::vector<std::string> MyStrings;   //! A vector of strings
  std::map<std::string, double> MyMap;  //! A map of pairs string->double
  MySimpleT MySimple;                   //! A simple wrapping type
  MyTemplateT<int> MyCustom;            //! A complex type from a class template

  /* IMPORTANT: The configurable<> needs a name() function to be available.
   * A static assertion is present, and will complain if unavailable. */
  const char* name() const { return "MyConfigurableClass"; }

  /* The function where JSON values are written to member variables. */
  void configure() {
    /* To instruct which key from JSON is supposed to be written to which
     * variable, use the bind_and_describe_data function. It takes the key, and
     * an lvalue reference to the variable. */
    bind_and_describe_data("MyStrings", MyStrings);
    bind_and_describe_data("MyMap", MyMap);
    bind_and_describe_data("MySimple", MySimple);
    bind_and_describe_data("MyCustom", MyCustom);
  }

  /* Custom function for demonstration purposes. */
  void print() {
    fmt::print("\nMyConfigurableClass:\n");
    fmt::print("├ MyStrings:     {}\n", MyStrings);
    fmt::print("├ MyMap:         {}\n", MyMap);
    fmt::print("├ MySimple:      {}\n", MySimple.value_);
    fmt::print("└ MyCustom:\n");
    fmt::print("  ├ value:       {}\n", MyCustom.value_);
    fmt::print("  └ description: {}\n", MyCustom.description_);
  }
};

static const bool USE_TEMPLATE_HELPER{false};

void section(const std::string& text) {
  fmt::print("\n{} {}\n", std::string('-', 78 - text.size()), text);
}

int main() {
  /* Create a JSON object using the '_json' literal. */
  auto j = R"(
  { "MyConfigurableClass": {
    "MyStrings": ["String1", "String2"],
    "MyMap": { "Key1": 1, "Key2": 2},
    "MySimple": 1,
    "MyCustom": { "value": 10.1e-2, "description": "My custom type"}
  }})"_json;

  section("Provided JSON object");

  fmt::print("{}\n", R"(
  { "MyConfigurableClass": {
    "MyStrings": ["String1", "String2"],
    "MyMap": { "Key1": 1, "Key2": 2},
    "MySimple": 1,
    "MyCustom": { "value": 10.1e-2, "description": "My custom type"}
  }})");

  /* Create an instance of the custom class. */
  MyConfigurableClass instance;

  section("Before configuring");

  /* Before configuring, the instance's internal JSON should be a nullptr. */
  fmt::print("\nInstance's internal JSON reference is null: {}\n",
             instance.get_json_const_ref().is_null());

  /* The printed values will be uninitialised. */
  instance.print();

  /* To configure the instance, use the function template helper, or the
   * set_json() and configure() member functions. */
  if constexpr (USE_TEMPLATE_HELPER) {
    configure(j, instance);
  } else {
    instance.set_json(j);
    instance.configure();
  }

  section("After configuring");

  fmt::print("\nInstance's internal JSON reference is null: {}\n",
             instance.get_json_const_ref().is_null());

  instance.print();

  return 0;
}
