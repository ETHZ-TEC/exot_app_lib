/**
 * @file meters/base_shared_memory.h
 * @author     Bruno Klopott
 * @brief      Measures contention to the hardware random number generator.
 */

#pragma once

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <memory>
#include <random>
#include <thread>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>

#if defined(__x86_64__)
#include <thread>

#include <exot/primitives/cache.h>
#include <exot/primitives/x86_64.h>
#endif

#if defined(__arm__) || defined(__aarch64__)
#include <exot/utilities/timing_source.h>
#endif

#include <exot/meters/base.h>
#include <exot/utilities/allocator.h>
#include <exot/utilities/cache.h>
#include <exot/utilities/configuration.h>
#include <exot/utilities/filesystem.h>
#include <exot/utilities/mmap.h>
#include <exot/utilities/pagemap.h>

#ifndef meter_shared_memory_base_CLEVER_FOR_LOOP
#define meter_shared_memory_base_CLEVER_FOR_LOOP true
#endif

#ifndef meter_shared_memory_base_SHUFFLED_ACCESS
#define meter_shared_memory_base_SHUFFLED_ACCESS true
#endif

namespace exot::modules {

/**
 * @brief      Base class for meters using shared memory.
 */
struct meter_shared_memory_base : module {
  using return_type = std::vector<std::uint64_t>;

  /**
   * @brief      Performs an action on an address from the address set
   * @note       Must be implemented by deriving classes.
   *
   * @param      address  The address
   *
   * @return     Cycles measurement
   */
  virtual std::uint64_t perform_action_on_address(void* address) = 0;

  /**
   * @brief      Returns the specific meter's name
   * @note       This function is used because we rarely redefine the 'settings'
   *             structure, which usually contains the meter name. Creating new
   *             settings would increase the amount of boilerplate.
   *
   * @return     The name
   */
  virtual const char* name() const = 0;

  struct options {
    static const bool clever_for_loop =
        meter_shared_memory_base_CLEVER_FOR_LOOP;
    static const bool shuffled_access =
        meter_shared_memory_base_SHUFFLED_ACCESS;
  };

  /**
   * @brief      Settings supported by the module
   */
  struct settings : public exot::utilities::configurable<settings> {
    std::string shm_file;           //! shared memory file
    bool use_huge_pages    = true;  //! use huge pages?
    unsigned set_count     = 1u;   //! number of LLC sets used for communication
    unsigned set_increment = 64u;  //! number of cache-line-sized spaces
                                   //! between consecutive  communication sets,
                                   //! default: page offsets (64 * line = 4096)

    using cache_maps_t =
        std::vector<typename exot::utilities::CacheInfo::map_type>;
    std::optional<cache_maps_t> cache_info;

    const char* name() const { return "cache"; }
    void configure() {
      bind_and_describe_data(
          "shm_file", shm_file,
          "the shared memory file |str|, e.g. /mnt/shm/file");
      bind_and_describe_data("use_huge_pages", use_huge_pages,
                             "map memory with the huge pages flag? |bool|");
      bind_and_describe_data(
          "set_count", set_count,
          "the number of sets to use |uint|, in range [1, 64]");
      bind_and_describe_data("set_increment", set_increment,
                             "choose every nth set |uint|, e.g. 16");
      bind_and_describe_data("cache_info", cache_info,
                             "manual cache info maps |CacheInfo[]|");
    }
  };

  /**
   * @brief      Constructs the meter module
   *
   * @param      conf  The module settings
   */
  explicit meter_shared_memory_base(settings& conf) : conf_{conf} {
    using namespace exot::utilities;

    llc_ = std::make_unique<CacheInfo>(
        (conf_.cache_info.has_value()
             ? std::make_unique<CPUCacheInfo>(0, conf_.cache_info.value())
             : std::make_unique<CPUCacheInfo>(0))
            ->llc());

    if (conf_.shm_file.empty())
      throw std::logic_error("Shared memory file must be provided!");
    if (!exot::utilities::exists(conf_.shm_file))
      throw std::logic_error("Shared memory file must exist!");
    if (!exot::utilities::is_readable(conf_.shm_file))
      throw std::logic_error("Shared memory file must be readable!");
    if (conf_.set_count == 0)
      throw std::logic_error("At least 1 set is needed for the channel!");
    if (conf_.set_increment < 1)
      throw std::logic_error("Set increment must be at least 1!");
    if (conf_.set_increment * conf_.set_count >
        llc_->number_of_sets().value()) {
      throw std::logic_error(
          "Set increment * set count cannot exceed the available number of "
          "sets!");
    }

    try {
      /* Defined here because type deduction of the initialiser lists will fail
       * in the make_unique helper. */
      auto prot =
          std::initializer_list<MMapProt>{MMapProt::Read, MMapProt::Write};
      auto flag = conf_.use_huge_pages
                      ? std::initializer_list<MMapFlag>{MMapFlag::Shared,
                                                        MMapFlag::HugeTable}
                      : std::initializer_list<MMapFlag>{MMapFlag::Shared};
      /* Unique pointer is used due to the lack of a default constructor. May
       * throw if unsuccessful. */
      mapping_ = std::make_unique<MMap>(conf_.shm_file, prot, flag);
    } catch (const std::exception& e) {
      throw std::logic_error(fmt::format(
          "Mapping shared memory failed.{}\n   why():  {}",
          conf_.use_huge_pages ? " Make sure to provide a huge pages "
                                 "shared memory, e.g. via hugetlbfs."
                               : "",
          e.what()));
    }

    /* Advice the kernel that the memory will be accessed randomly, hoping that
     * would prevent unintended prefetching of consecutive addresses. */
    advise(mapping_->get_address(), mapping_->get_length(), Advice::random);

    decomposer_ = std::make_unique<AddressDecomposer>(*llc_);

    base_       = reinterpret_cast<std::uintptr_t>(mapping_->get_address());
    auto offset = static_cast<std::uintptr_t>(0ull);

    debug_log_->debug(
        "[meter] created mapping of size: {:#x} from address: {:#x} "
        "(physical: {:#x})",
        mapping_->get_length(), base_,
        resolver_(reinterpret_cast<void*>(base_)));

#if defined(__x86_64__)
    /* Only hyper-threaded multicore platforms are supported, hence '/ 2'. */
    const auto core_count = std::thread::hardware_concurrency() / 2;
#endif

    for (auto i = 0u; i < conf_.set_count; ++i) {
      auto addr      = base_ + offset;
      auto phys      = resolver_(reinterpret_cast<void*>(addr));
      auto set_index = decomposer_->get_set(phys);
#if defined(__x86_64__)
      auto slice_index =
          exot::primitives::slice_selection_hash(phys, core_count);
#else
      auto slice_index = -1;
#endif

      debug_log_->debug(
          "[meter] adding address {:#x} ({:#x}) to address set, offset from "
          "base: {:#8x}, set/index: {:#6x}"
#if defined(__x86_64__)
          ", slice: {:#b}"
#endif
          ,
          addr, phys, offset, set_index
#if defined(__x86_64__)
          ,
          slice_index
#endif
      );

      address_set_.push_back(reinterpret_cast<void*>(addr));
      address_set_info_.emplace_back(reinterpret_cast<void*>(addr), set_index,
                                     slice_index);
      offset += llc_->coherency_line_size().value() * conf_.set_increment;
    }

    readings_.resize(address_set_.size());

    /* Initialise index array. */
    indeces_.resize(address_set_.size());
    std::iota(indeces_.begin(), indeces_.end(), 0);
  }

  /**
   * @brief      Perform module measurement
   * @note       Either access the address set sequentially, or in random
   *             fashion using shuffled indeces. No penalty due to the shuffle
   *             and index access has been observed.
   *
   * @return     The result of actions on each address in the address set
   */
  return_type measure() {
    if constexpr (options::shuffled_access) {
      /* Shuffle the indeces array. */
      std::shuffle(indeces_.begin(), indeces_.end(), rd_gen_);

      /* Access using shuffled indeces */
      for (auto i : indeces_) {
        readings_[i] = this->perform_action_on_address(address_set_[i]);
      }
    } else {
      for (auto i = 0; i < address_set_.size(); ++i) {
        readings_[i] = this->perform_action_on_address(address_set_[i]);
      }
    }

    return readings_;
  }

  /**
   * @brief      Gets the address set.
   *
   * @return     The address set.
   */
  const auto& get_address_set() const { return address_set_; }

  /**
   * @brief      Gets the address set information (address, set, slice).
   *
   * @return     The address set information.
   */
  const auto& get_address_set_info() const { return address_set_info_; }

  /**
   * @brief      Get descriptions of return values
   * @details    The descriptions contain variable names and units
   *
   * @return     A vector with descriptions
   */
  std::vector<std::string> header() {
    std::vector<std::string> descriptions;

    for (auto i = 0; i < address_set_.size(); ++i) {
      descriptions.push_back(exot::utilities::generate_header(
          this->name(), "access_time", i, "cycles"));
    }

    return descriptions;
  }

 protected:
  using logger_pointer = std::shared_ptr<spdlog::logger>;
  logger_pointer debug_log_ =
      spdlog::get("log") ? spdlog::get("log") : spdlog::default_logger();

  settings conf_;
  return_type readings_;

  std::unique_ptr<exot::utilities::MMap> mapping_;
  std::uintptr_t base_;

  using address_type      = void*;
  using address_info_type = std::tuple<address_type, int, int>;

  std::vector<address_type> address_set_;
  std::vector<address_info_type> address_set_info_;
  std::vector<unsigned char> indeces_;

  std::random_device rd_;
  std::mt19937 rd_gen_{rd_()};

  std::unique_ptr<exot::utilities::CacheInfo> llc_ = nullptr;
  exot::utilities::AddressTranslator resolver_;
  std::unique_ptr<exot::utilities::AddressDecomposer> decomposer_ = nullptr;
};

}  // namespace exot::modules
