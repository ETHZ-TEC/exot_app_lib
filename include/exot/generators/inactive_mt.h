/**
 * @file generators/utilisation_mt.h
 * @author     Bruno Klopott
 * @brief      Busy floating-point loop multi-threaded load generator.
 */

#pragma once

#include <thread>

#include <exot/generators/base_bitset.h>

namespace exot::modules {

/**
 * @brief Inactive multithreaded generator
 *
 */
struct generator_inactive_mt : exot::modules::bitset_generator_base {
  using exot::modules::bitset_generator_base::bitset_generator_base;

  void generate_load(const decomposed_type& decomposed_subtoken,
                     const enable_flag_type& flag, core_type core,
                     index_type index) {
    std::this_thread::yield();
  }
};

}  // namespace exot::modules
