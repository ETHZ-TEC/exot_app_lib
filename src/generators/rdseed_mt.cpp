#if defined(__x86_64__)

#include <exot/generators/rdseed_mt.h>

#include <spdlog/spdlog.h>

#include <exot/primitives/rng.h>

using namespace exot::modules;

void generator_rdseed_mt::generate_load(
    const generator_rdseed_mt::decomposed_type& decomposed_subtoken,
    const generator_rdseed_mt::enable_flag_type& flag,
    generator_rdseed_mt::core_type core,
    generator_rdseed_mt::index_type index) {
  if (decomposed_subtoken) {
    SPDLOG_LOGGER_TRACE(
        logger_, "[generator->generate_load] worker on core {} enabled", core);
    /* Create contention to the hardware random number generator. */
    while (flag) {
      exot::primitives::seed64();
      exot::primitives::seed64();
      exot::primitives::seed64();
      exot::primitives::seed64();
      exot::primitives::seed64();
    }
  }
}

#endif
