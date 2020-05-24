#include <exot/generators/utilisation_mt.h>

#include <spdlog/spdlog.h>

using namespace exot::modules;

void generator_utilisation_mt::generate_load(
    const generator_utilisation_mt::decomposed_type& decomposed_subtoken,
    const generator_utilisation_mt::enable_flag_type& flag,
    generator_utilisation_mt::core_type core,
    generator_utilisation_mt::index_type index) {
  volatile double a = 2.0e0;
  volatile double b = 2.0e0;

  if (decomposed_subtoken) {
    SPDLOG_LOGGER_TRACE(
        logger_, "[generator->generate_load] worker on core {} enabled", core);
    /* Heavy floating-point work loop. */
    while (flag) {
      a *= b;
      b -= a;
    }
  }
}
