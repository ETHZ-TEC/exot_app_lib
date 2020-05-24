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
 * @file utilities/configuration.h
 * @author     Bruno Klopott
 * @brief      General purpose utilities for configuration/bootstrapping.
 */

#pragma once

#include <algorithm>
#include <chrono>
#include <map>
#include <numeric>
#include <optional>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include <fmt/format.h>
#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

#include <exot/utilities/helpers.h>
#include <exot/utilities/thread.h>
#include <exot/utilities/types.h>

namespace exot::utilities {

/**
 * The pointer to the root config, has to form a valid JSON pointer or be empty.
 */
#ifndef POINTER_TO_ROOT_CONFIG
#define POINTER_TO_ROOT_CONFIG ""
#endif

static const char* CONFIG_ROOT = POINTER_TO_ROOT_CONFIG;

/**
 * @brief      Base class for configurable types
 * @note       Empty base class does not increase the object size. The structure
 *             is used mostly for type checks.
 */
struct configurable_base {};

/**
 * @brief      Type trait which determines whether a type has a settings member
 *             type, and component and type member functions.
 * @todo       This type trait is incomplete (does not detect all required
 *             members).
 *
 * @tparam     T     The type
 * @tparam     _     Template helper
 */
template <typename T, typename = void, typename = void>
struct is_configurable : std::false_type {};

template <typename T>
struct is_configurable<
    T, std::enable_if_t<std::is_base_of_v<configurable_base, T>>,
    std::void_t<decltype(std::declval<std::decay_t<T>>().name()),
                decltype(std::declval<std::decay_t<T>>().configure())>>
    : std::true_type {};

template <typename T>
inline constexpr bool is_configurable_v = is_configurable<T>::value;

/**
 * @brief      Attempts to assign a value from a json object to a referenced
 *             variable, using the provided key.
 *
 * @param[in]  json       The json object
 * @param[in]  key        The key
 * @param      ref        The reference to data
 *
 * @tparam     T          The type of data
 * @tparam     <unnamed>  Template helper
 */

/* @todo Temporarily provide manual partial template specialisations until the
 * issue with the following declaration is resolved. It does not cause issues
 * when provided in isolation from the library.

template <typename T, typename K,
          typename = std::enable_if_t<(
            exot::utilities::is_same_d<std::decay_t<K>, std::string>::value ||
            exot::utilities::is_same_d<K,
nlohmann::json::json_pointer>::value)>>

 */

template <typename T, typename K>
inline void get_from_json_to(const nlohmann::json& json, K key, T& ref);

/**
 * @brief      Temporary partial specialisation for get_from_json_to and const
 *             char* key.
 */
template <typename T>
inline void get_from_json_to(const nlohmann::json& json, const char* key,
                             T& ref) {
  if constexpr (is_iterable_and_clearable<decltype(ref)>::value) {
    /* ---------------------------------------------------------------------- */
    /* If T is a type that can be iterated and cleared, likely a container. */

    /* Create an optional storage for the old value of ref. If ref was not
     * empty, store its old value. */
    std::optional<T> old_value;
    if (!ref.empty()) old_value = ref;

    try {
      /* Clear the container to prevent appending if a default value was set. If
       * this operation proves unsuccessful, the value will be restored. */
      if (!ref.empty()) ref.clear();
      json.at(key).get_to(ref);
    } catch (const nlohmann::json::out_of_range& e) {
      /* Ignore key-not-found and unresolved reference token errors and restore
       * the erased value if possible. */
      if (is_one_of(e.id, 403, 404)) {
        if (old_value.has_value()) ref = old_value.value();
      } else {
        throw nlohmann::json::out_of_range::create(
            e.id,
            e.what() + fmt::format(", the accessed key was: \"{}\"", key));
      }
    } catch (const nlohmann::json::type_error& e) {
      if (e.id == 302) {
        /* Only allow a type mismatch error when a value has been set to null.
         * Otherwise, rethrow with info about the accessed key. */
        if (json.at(key).type() != nlohmann::json::value_t::null) {
          /* json::json_pointer defines a std::string() operator, hence it's not
           * necessary to explicitly convert it below. */
          throw nlohmann::json::type_error::create(
              e.id,
              e.what() + fmt::format(", the accessed key was: \"{}\"", key));
        } else {
          if (old_value.has_value()) { ref = old_value.value(); }
        }
      } else {
        throw nlohmann::json::type_error::create(
            e.id,
            e.what() + fmt::format(", the accessed key was: \"{}\"", key));
      }
    }
  } else {
    /* ---------------------------------------------------------------------- */
    /* If T is a simpler type. */
    try {
      json.at(key).get_to(ref);
    } catch (const nlohmann::json::out_of_range& e) {
      /* Only ignore key-not-found and unresolved reference errors. */
      if (is_none_of(e.id, 403, 404)) {
        throw nlohmann::json::out_of_range::create(
            e.id,
            e.what() + fmt::format(", the accessed key was: \"{}\"", key));
      }
    } catch (const nlohmann::json::type_error& e) {
      /* Only allow a type mismatch error when a value has been set to null.
       * Otherwise, rethrow with info about the accessed key. */
      if (e.id == 302) {
        if (json.at(key).type() != nlohmann::json::value_t::null) {
          throw nlohmann::json::type_error::create(
              e.id,
              e.what() + fmt::format(", the accessed key was: \"{}\"", key));
        }
      } else {
        throw nlohmann::json::type_error::create(
            e.id,
            e.what() + fmt::format(", the accessed key was: \"{}\"", key));
      }
    }
  }
}

/**
 * @brief      Temporary partial specialisation for get_from_json_to and a JSON
 *             pointer.
 */
template <typename T>
inline void get_from_json_to(const nlohmann::json& json,
                             const nlohmann::json::json_pointer& key, T& ref) {
  if constexpr (is_iterable_and_clearable<decltype(ref)>::value) {
    /* ---------------------------------------------------------------------- */
    /* If T is a type that can be iterated and cleared, likely a container. */

    /* Create an optional storage for the old value of ref. If ref was not
     * empty, store its old value. */
    std::optional<T> old_value;
    if (!ref.empty()) old_value = ref;

    try {
      /* Clear the container to prevent appending if a default value was set. If
       * this operation proves unsuccessful, the value will be restored. */
      if (!ref.empty()) ref.clear();
      json.at(key).get_to(ref);
    } catch (const nlohmann::json::out_of_range& e) {
      /* Ignore key-not-found and unresolved reference token errors and restore
       * the erased value if possible. */
      if (is_one_of(e.id, 403, 404)) {
        if (old_value.has_value()) ref = old_value.value();
      } else {
        throw nlohmann::json::out_of_range::create(
            e.id,
            e.what() + fmt::format(", the accessed key was: \"{}\"", key));
      }
    } catch (const nlohmann::json::type_error& e) {
      if (e.id == 302) {
        /* Only allow a type mismatch error when a value has been set to null.
         * Otherwise, rethrow with info about the accessed key. */
        if (json.at(key).type() != nlohmann::json::value_t::null) {
          /* json::json_pointer defines a std::string() operator, hence it's not
           * necessary to explicitly convert it below. */
          throw nlohmann::json::type_error::create(
              e.id,
              e.what() + fmt::format(", the accessed key was: \"{}\"", key));
        } else {
          if (old_value.has_value()) { ref = old_value.value(); }
        }
      } else {
        throw nlohmann::json::type_error::create(
            e.id,
            e.what() + fmt::format(", the accessed key was: \"{}\"", key));
      }
    }
  } else {
    /* ---------------------------------------------------------------------- */
    /* If T is a simpler type. */
    try {
      json.at(key).get_to(ref);
    } catch (const nlohmann::json::out_of_range& e) {
      /* Only ignore key-not-found and unresolved reference errors. */
      if (is_none_of(e.id, 403, 404)) {
        throw nlohmann::json::out_of_range::create(
            e.id,
            e.what() + fmt::format(", the accessed key was: \"{}\"", key));
      }
    } catch (const nlohmann::json::type_error& e) {
      /* Only allow a type mismatch error when a value has been set to null.
       * Otherwise, rethrow with info about the accessed key. */
      if (e.id == 302) {
        if (json.at(key).type() != nlohmann::json::value_t::null) {
          throw nlohmann::json::type_error::create(
              e.id,
              e.what() + fmt::format(", the accessed key was: \"{}\"", key));
        }
      } else {
        throw nlohmann::json::type_error::create(
            e.id,
            e.what() + fmt::format(", the accessed key was: \"{}\"", key));
      }
    }
  }
}

/**
 * @brief      Mixin for use in CRTP patterns to introduce configurability to
 *             compatible classes.
 * @details    To take advantage of the configurable mixin the class of type D
 *             should have a function returning a string: name().
 *             These are used to subset configuration structures using JSON
 *             pointers.
 * @todo Provide a way for this to return a clipp::group for CLI config
 *
 * @tparam     D     The type to be mixed-in.
 */
template <typename D>
struct configurable : configurable_base {
  const char* name() const {
    static_assert(is_configurable<D>::value,
                  "The class should have a name() member function.");
    return derived().name();
  }

  /**
   * @brief      Sets the internal JSON config
   *
   * @param[in]  root  The root JSON file/config
   */
  void set_json(const nlohmann::json& root) {
    try {
      json_config_ = root.at(get_json_pointer());
    } catch (const nlohmann::json::out_of_range& e) { json_config_ = nullptr; }
  }

  /**
   * @brief      Gets the internal JSON config's value
   *
   * @return     The value of the JSON
   */
  nlohmann::json get_json() const { return json_config_; }

  /**
   * @brief      Gets the internal JSON config as a constant reference.
   *
   * @return     The constant reference to the JSON.
   */
  const nlohmann::json& get_json_const_ref() const { return json_config_; }

  /**
   * @brief      Configure, to be implemented in the derived class
   */
  void configure();

  std::string describe() {
    auto holder = fmt::format("[{}]\n", name());
    derived().configure();
    for (auto const& [k, v] : descriptions_) {
      holder.append(fmt::format("| {:<40s}  {}\n", k, v));
    }
    return holder;
  }

 protected:
  /**
   * @brief      CRTP helper for casting this class to the CRTP-derived class
   *
   * @return     The reference to this using the pointer to the base class
   */
  constexpr D& derived() { return *static_cast<D*>(this); }

  /**
   * @brief      CRTP helper for casting this class to the CRTP-derived class
   *
   * @return     The const reference to this casted to the base class
   */
  constexpr const D& derived() const { return *static_cast<const D*>(this); }

  /**
   * @brief      Gets a JSON pointer to the class-specific configuration.
   *
   * @return     The JSON pointer to configuration.
   */
  nlohmann::json::json_pointer get_json_pointer() const {
    return nlohmann::json::json_pointer(
        fmt::format("{}/{}", CONFIG_ROOT, name()));
  }

  /**
   * @brief      Gets a reference to the contained JSON config
   *
   * @return     The reference to the JSON config.
   */
  nlohmann::json& get_json_ref() { return json_config_; }

  /**
   * @brief      Bind a JSON value at a key to a local variable
   * @details    The function was refactored to allow incomplete configs to be
   *             used, in which case the JSON config was not assigned and the
   *             reference is a nullptr.
   *
   * @param[in]  key   The JSON key
   * @param      to    The reference to the local variable
   *
   * @tparam     T     The type of the local variable
   *
   * @return     True if able to bind, false otherwise.
   */
  template <typename T>
  bool bind_and_describe_data(const char* key, T& to,
                              std::string&& description = std::string{}) {
    descriptions_.insert_or_assign(key, description);
    if (get_json_const_ref() == nullptr) return false;
    static_assert(
        std::is_lvalue_reference<decltype(to)>::value,
        "bind_and_describe_data: 'to' must be an lvalue reference (T&).");
    get_from_json_to(get_json_const_ref(), key, std::forward<T&>(to));
    return true;
  }

 private:
  /**
   * The JSON config
   */
  nlohmann::json json_config_;
  std::map<const char*, std::string> descriptions_;
};

/* Base templates that need to be specialised. */

/**
 * @brief      Sets the JSON object in configurables
 *
 * @param[in]  json  The json
 * @param      ts    The configurables
 *
 * @tparam     U     The type in which the json object is provided
 * @tparam     Ts    The types of the configurables
 */
template <typename U, typename... Ts>
inline void set_json(U json, Ts&&... ts);

/**
 * @brief      Sets the JSON object in configurables and configures them
 *
 * @param[in]  json  The json
 * @param      ts    The configurables
 *
 * @tparam     U     The type in which the json object is provided
 * @tparam     Ts    The types of the configurables
 */
template <typename U, typename... Ts>
inline void configure(U json, Ts&&... ts);

/* Template specialisations with const reference to a JSON object. */

/**
 * @brief      Sets the JSON file in multiple configurables
 *
 * @param[in]  root       The root JSON
 * @param[in]  ts         The configurables
 *
 * @tparam     Ts         The types of the configurables
 */
template <typename... Ts>
inline void set_json(nlohmann::json::const_reference root, Ts&&... ts) {
  (..., ts.set_json(root));
}

/**
 * @brief      Configures multiple configurables
 *
 * @param[in]  ts         The configurables
 *
 * @tparam     Ts         The types of the configurables
 */
template <typename... Ts>
inline void only_configure(Ts&&... ts) {
  (..., ts.configure());
}

/**
 * @brief      Sets the JSON file in multiple configurables and configures them
 *
 * @param[in]  root       The root JSON
 * @param[in]  ts         The configurables
 *
 * @tparam     Ts         The types of the configurables
 */
template <typename... Ts>
inline void configure(nlohmann::json::const_reference root, Ts&&... ts) {
  set_json(root, ts...);
  only_configure(ts...);
}

/**
 * @brief      Sort and deduplicate a vector of comparable elements
 * @todo Move to a more general header file
 *
 * @param      vector  The vector
 *
 * @tparam     T       The sorted and deduplicated vector
 */
template <typename T>
void sort_and_deduplicate(std::vector<T>& vector) {
  std::sort(vector.begin(), vector.end());
  vector.erase(std::unique(vector.begin(), vector.end()), vector.end());
}

/**
 * @brief      Bootstrap a vector of cores
 * @details    Checks hardware concurrency, initialises the vector if empty, and
 *             checks if any of the specified cores are out of range.
 *
 * @param      cores  The vector with core numbers
 */
void bootstrap_cores(std::vector<unsigned>& cores);

template <typename T>
inline constexpr bool is_string_or_arithmetic = (is_string_d<T> ||
                                                 is_arithmetic_d<T>);

template <typename T>
inline constexpr bool is_string_or_arithmetic_or_pointer =
    (is_string_d<T> || is_arithmetic_d<T> || is_pointer_d<T>);

/**
 * @brief      Generate a header for application logging purposes
 *
 * @param      ts    Values to supply to the header, exactly 4 have to be
 *                   provided. 1: module, 2: variable, 3: dimension, 4: unit.
 *
 * @tparam     Ts    Types of values, see static_assert's for compatibility
 *
 * @return     A string with a formatted header
 */
template <typename... Ts>
std::string generate_header(Ts&&... ts) {
  /* Make sure that the right number of arguments were provided. */
  static_assert(sizeof...(ts) == 4, "Only meaningful with 4 arguments");

  /* Forward as tuple to allow indexing. */
  auto t = std::forward_as_tuple(std::forward<Ts>(ts)...);

  /* Verify that types conform to design. */
  /* 1. The first argument should be the output of settings::name(), or the
   *    module name. */
  static_assert(is_string_d<decltype(std::get<0>(t))>,
                "First argument must be convertible to a string");

  /* 2. The second argument is the variable being produced, can be a name or
   *    a number. */
  static_assert(
      is_string_or_arithmetic<decltype(std::get<1>(t))>,
      "Second argument must be convertible to a string or an arithmetic type");

  /* 3. The third argument is the dimension of the variable, e.g. which core
   *    is being represented. It can also be a pointer, to account for
   *    addresses, which will be casted to (void*). */
  static_assert(is_string_or_arithmetic_or_pointer<decltype(std::get<2>(t))>,
                "Third argument must be convertible to a string or an "
                "arithmetic type or a pointer type");

  /* 4. The fourth argument is the unit of the variable, e.g. Hz, Watts. */
  static_assert(is_string_d<decltype(std::get<3>(t))>,
                "Fourth argument must be convertible to a string");

  /* Caveat: A (char*) type, although satisfying the is_pointer_d trait, is also
   * convertible to string, therefore will be interpreted as a string.
   * To get an address of a char type pass your argument at the calling site
   * to fmt::ptr(), casting it to (void*), or perform the casting yourself. */
  const auto _3rd = [&](auto&& x) {
    if constexpr (is_pointer_d<decltype(x)> && !is_string_d<decltype(x)>) {
      return fmt::ptr(x);
    } else {
      return x;
    }
  };

  return fmt::format(FMT_STRING("{}:{}:{}:{}"), std::get<0>(t), std::get<1>(t),
                     _3rd(std::get<2>(t)), std::get<3>(t));
}

}  // namespace exot::utilities

namespace nlohmann {

/**
 * @brief      Overload of the json serialiser for std::optional wrapped types
 *
 * @tparam     T     The type "carried" by std::optional
 */
template <typename T>
struct adl_serializer<std::optional<T>> {
  static void to_json(json& j, const std::optional<T>& o) {
    if (o.has_value()) {
      j = o.value();
    } else {
      j = nullptr;
    }
  }

  static void from_json(const json& j, std::optional<T>& o) {
    if (j.is_null()) {
      o = std::nullopt;
    } else {
      o = j.get<T>();
    }
  }
};

/**
 * @brief      Overload of the json serialiser for std::chrono::duration.
 * @details    Values are written to and from numeric values in seconds. For
 *             example, to write a nanosecond to a duration object, write 1e-9
 *             in the JSON file. The target type might not be able to hold the
 *             specified duration!
 *
 * @tparam     Rep     Underlying type representing the duration object.
 * @tparam     Period  A std::ratio representing the tick period.
 */
template <typename Rep, typename Period>
struct adl_serializer<std::chrono::duration<Rep, Period>> {
  static void to_json(json& j, const std::chrono::duration<Rep, Period>& o) {
    auto d = std::chrono::duration_cast<
        std::chrono::duration<double, std::ratio<1>>>(o);
    j = d.count();
  }

  static void from_json(const json& j, std::chrono::duration<Rep, Period>& o) {
    if (!j.is_number()) {
      throw json::type_error::create(
          302,
          fmt::format("type must be a number, but was a {}", j.type_name()));
    } else {
      auto d = std::chrono::duration<double, std::ratio<1>>(
          j.get<json::number_float_t>());
      o = std::chrono::duration_cast<std::chrono::duration<Rep, Period>>(d);
    }
  }
};

/**
 * @brief      Overload of the json serialiser for SchedulingPolicy
 */
template <>
struct adl_serializer<exot::utilities::SchedulingPolicy> {
  static void to_json(json& j, const exot::utilities::SchedulingPolicy& o) {
    switch (o) {
      case exot::utilities::SchedulingPolicy::Other:
        j = "other";
        break;
      case exot::utilities::SchedulingPolicy::Fifo:
        j = "fifo";
        break;
      case exot::utilities::SchedulingPolicy::RoundRobin:
        j = "round_robin";
        break;
      case exot::utilities::SchedulingPolicy::Deadline:
        j = "deadline";
        break;
      case exot::utilities::SchedulingPolicy::Batch:
        j = "batch";
        break;
      case exot::utilities::SchedulingPolicy::Idle:
        j = "idle";
        break;
      default:
        break;
    }
  }

  static void from_json(const json& j, exot::utilities::SchedulingPolicy& o) {
    if (!j.is_string()) {
      throw json::type_error::create(
          302,
          fmt::format("type must be a string, but was a {}", j.type_name()));
    } else {
      auto value = j.get<json::string_t>();
      if (value == "other") {
        o = exot::utilities::SchedulingPolicy::Other;
      } else if (value == "fifo") {
        o = exot::utilities::SchedulingPolicy::Fifo;
      } else if (value == "round_robin") {
        o = exot::utilities::SchedulingPolicy::RoundRobin;
      } else if (value == "deadline") {
        o = exot::utilities::SchedulingPolicy::Deadline;
      } else if (value == "batch") {
        o = exot::utilities::SchedulingPolicy::Batch;
      } else if (value == "idle") {
        o = exot::utilities::SchedulingPolicy::Idle;
      } else {
        throw json::other_error::create(
            501, fmt::format("provided scheduling policy value \"{}\" is not "
                             "featured in the enum class",
                             value));
      }
    }
  }
};

/**
 * @brief      Overload of the json serialiser for SPDLOG's log level
 */
template <>
struct adl_serializer<spdlog::level::level_enum> {
  static void to_json(json& j, const spdlog::level::level_enum& o) {
    switch (o) {
      case spdlog::level::level_enum::trace:
        j = "trace";
        break;
      case spdlog::level::level_enum::debug:
        j = "debug";
        break;
      case spdlog::level::level_enum::info:
        j = "info";
        break;
      case spdlog::level::level_enum::warn:
        j = "warn";
        break;
      case spdlog::level::level_enum::err:
        j = "err";
        break;
      case spdlog::level::level_enum::critical:
        j = "critical";
        break;
      case spdlog::level::level_enum::off:
        j = "off";
        break;
      default:
        break;
    }
  }

  static void from_json(const json& j, spdlog::level::level_enum& o) {
    if (!j.is_string()) {
      throw json::type_error::create(
          302,
          fmt::format("type must be a string, but was a {}", j.type_name()));
    } else {
      auto value = j.get<json::string_t>();
      if (value == "trace") {
        o = spdlog::level::level_enum::trace;
      } else if (value == "debug") {
        o = spdlog::level::level_enum::debug;
      } else if (value == "info") {
        o = spdlog::level::level_enum::info;
      } else if (value == "warn") {
        o = spdlog::level::level_enum::warn;
      } else if (value == "err") {
        o = spdlog::level::level_enum::err;
      } else if (value == "critical") {
        o = spdlog::level::level_enum::critical;
      } else if (value == "off") {
        o = spdlog::level::level_enum::off;
      } else {
        throw json::other_error::create(
            501, fmt::format("provided scheduling policy value \"{}\" is not "
                             "featured in the enum class",
                             value));
      }
    }
  }
};

}  // namespace nlohmann
