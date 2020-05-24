/**
 * @file utilities/configuration.cpp
 * @author     Bruno Klopott
 * @brief      Implementation of the bootstrapping utilities from
 *             @ref configuration.h.
 */

#include <exot/utilities/configuration.h>

namespace exot::utilities {

void bootstrap_cores(std::vector<unsigned>& cores) {
  auto concurrency = std::thread::hardware_concurrency();

  if (cores.empty()) {
    cores.resize(concurrency);
    std::iota(cores.begin(), cores.end(), 0);
  }

  /* Make sure that no provided core exceeds available core number.
   */
  if (std::any_of(cores.begin(), cores.end(),
                  [=](auto el) { return el >= concurrency; })) {
    throw std::out_of_range(
        "At least one element in the CPU vector exceeds hardware "
        "concurrency.");
  }

  /* Sort and remove duplicates from the vector of cores
   */
  sort_and_deduplicate(cores);
}

}  // namespace exot::utilities
