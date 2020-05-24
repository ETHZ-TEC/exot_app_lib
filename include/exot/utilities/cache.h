/**
 * @file utilities/cache.h
 * @author     Bruno Klopott
 * @brief      Cache identification utility.
 */

#pragma once

#include <array>
#include <cmath>
#include <map>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>

#include <fmt/format.h>

#include <exot/utilities/bits.h>
#include <exot/utilities/filesystem.h>
#include <exot/utilities/formatting.h>
#include <exot/utilities/types.h>

/**
 * Accesses values defined in sysfs. For example, for the CPU directory
 * /sys/devices/system/cpu/cpu0/ the values are (Intel(R) Core(TM) i7-6700):
 *
 * ./cache/index0/physical_line_partition  1
 * ./cache/index0/number_of_sets           64
 * ./cache/index0/ways_of_associativity    8
 * ./cache/index0/id                       0
 * ./cache/index0/shared_cpu_list          0,4
 * ./cache/index0/type                     Data
 * ./cache/index0/size                     32K
 * ./cache/index0/level                    1
 * ./cache/index0/coherency_line_size      64
 * ./cache/index0/shared_cpu_map           11
 * ./cache/index1/physical_line_partition  1
 * ./cache/index1/number_of_sets           64
 * ./cache/index1/ways_of_associativity    8
 * ./cache/index1/id                       0
 * ./cache/index1/shared_cpu_list          0,4
 * ./cache/index1/type                     Instruction
 * ./cache/index1/size                     32K
 * ./cache/index1/level                    1
 * ./cache/index1/coherency_line_size      64
 * ./cache/index1/shared_cpu_map           11
 * ./cache/index2/physical_line_partition  1
 * ./cache/index2/number_of_sets           1024
 * ./cache/index2/ways_of_associativity    4
 * ./cache/index2/id                       0
 * ./cache/index2/shared_cpu_list          0,4
 * ./cache/index2/type                     Unified
 * ./cache/index2/size                     256K
 * ./cache/index2/level                    2
 * ./cache/index2/coherency_line_size      64
 * ./cache/index2/shared_cpu_map           11
 * ./cache/index3/physical_line_partition  1
 * ./cache/index3/number_of_sets           8192
 * ./cache/index3/ways_of_associativity    16
 * ./cache/index3/id                       0
 * ./cache/index3/shared_cpu_list          0-7
 * ./cache/index3/type                     Unified
 * ./cache/index3/size                     8192K
 * ./cache/index3/level                    3
 * ./cache/index3/coherency_line_size      64
 * ./cache/index3/shared_cpu_map           ff
 */

namespace exot::utilities {

/**
 * @brief      Cache type enumeration
 */
enum class CacheType : int {
  Data        = 0,
  Instruction = 1,
  Unified     = 2,
  Unknown     = -1
};

/**
 * @brief      Cache info structure, accessed per core and cache index
 */
struct CacheInfo {
  using key_type   = std::string;                     //! mapping key type
  using value_type = std::optional<int>;              //! mapping value type
  using map_type   = std::map<key_type, value_type>;  //! mapping type

  /**
   * No default constructor
   */
  CacheInfo() = delete;

  /**
   * @brief      Manually assigns cache info for a cpu & index pair.
   * @note       The provided mapping must have all keys already present in the
   *             empty `cache_params_`.
   *
   * @param[in]  cpu             The cpu
   * @param[in]  index           The index
   * @param[in]  manual_mapping  The manual mapping
   */
  CacheInfo(unsigned cpu, unsigned index, const map_type& manual_mapping)
      : cpu_{cpu}, index_{index} {
    /* Checks if a key 'key' is present in mapping 'map'. */
    auto contains = [](const key_type& key, const map_type& map) -> bool {
      return map.find(key) != map.end();
    };

    /* Make sure that all cache_params_ keys are present in the mapping. */
    for (const auto& [key, _] : cache_params_) {
      if (!contains(key, manual_mapping)) {
        throw std::logic_error(
            fmt::format("At least 1 key required by CacheInfo missing in the "
                        "provided mapping: {}",
                        key));
      } else {
        cache_params_.at(key) = manual_mapping.at(key);
      }
    }
  }

  /**
   * @brief      Get the cache info for a cpu & index pair.
   *
   * @param[in]  cpu    The cpu
   * @param[in]  index  The index
   */
  explicit CacheInfo(unsigned cpu, unsigned index) : cpu_{cpu}, index_{index} {
    /* The sysfs cache info is not available on ARM platforms! */
    static constexpr auto source_fmt_ =
        "/sys/devices/system/cpu/cpu{}/cache/index{}/";
    const auto root_path_ = fmt::format(source_fmt_, cpu, index);

    /* The root path must be a valid directory. If it's not, it's either not
     * accessible, or does not exist (like in the case of ARM processors). */
    if (!is_directory(root_path_)) {
      throw std::logic_error(fmt::format(
          "the cpu-index pair ({}, {}) does not resolve to a valid path ({})",
          cpu, index, root_path_));
    }

    /* Loop through keys in cache_params_ and fill with values, if possible. */
    for (auto& [key, ___] : cache_params_) {
      auto value = value_type{std::nullopt};
      auto path  = root_path_ + key;

      if (key == "size") {
        /* "size" has a 'K' appended to it, extract just the number. */
        auto _ = exot::utilities::get_value_from_file<std::string>(path);
        if (_.has_value())
          value = kibibytes_to_bytes(
              static_cast<int>(exot::utilities::extract_number(_.value())));
      } else if (key == "type") {
        /* "type" returns a string, either Data, Instruction, or Unified. */
        auto _ = exot::utilities::get_value_from_file<std::string>(path);

        if (_.has_value()) {
          if (_.value() == "Data") {
            value = static_cast<int>(CacheType::Data);
          } else if (_.value() == "Instruction") {
            value = static_cast<int>(CacheType::Instruction);
          } else if (_.value() == "Unified") {
            value = static_cast<int>(CacheType::Unified);
          } else {
            value = static_cast<int>(CacheType::Unknown);
          }
        }

      } else if (key == "shared_cpu_map") {
        /* 'shared_cpu_map' is reported as a hexadecimal value-bitfield. */
        auto _ = exot::utilities::get_value_from_file<std::string>(path);

        if (_.has_value()) {
          value = exot::utilities::read_hex_value(_.value());
        }
      } else {
        value = exot::utilities::get_value_from_file<int>(path);
      }

      /* If no value has been set so far, a nullopt will be written. */
      cache_params_[key] = value;
    }
  }

  /**
   * @brief      Convert kibibytes to bytes.
   * @note       Sysfs provides values in KiB, e.g. 8192K is 8 MiB, not 8MB.
   *
   * @param[in]  KiB   The value in kibibytes
   *
   * @return     The corresponding bytes value.
   */
  constexpr static auto kibibytes_to_bytes(int KiB) -> int {
    return KiB * 1024;
  }

  // clang-format off
  const auto& get_params() const { return cache_params_; }
  inline unsigned cpu()           const { return cpu_; }
  inline unsigned index()         const { return index_; }
  inline value_type physical_line_partition() const { return cache_params_.at("physical_line_partition"); }
  inline value_type number_of_sets()          const { return cache_params_.at("number_of_sets"); }
  inline value_type ways_of_associativity()   const { return cache_params_.at("ways_of_associativity"); }
  inline value_type id()                      const { return cache_params_.at("id"); }
  inline value_type type()                    const { return cache_params_.at("type"); }
  inline value_type size()                    const { return cache_params_.at("size"); }
  inline value_type level()                   const { return cache_params_.at("level"); }
  inline value_type coherency_line_size()     const { return cache_params_.at("coherency_line_size"); }
  inline value_type shared_cpu_map()          const { return cache_params_.at("shared_cpu_map"); }
  // clang-format on

  bool is_valid() const {
    return level().has_value()                     //
           && type().has_value()                   //
           && size().has_value()                   //
           && number_of_sets().has_value()         //
           && ways_of_associativity().has_value()  //
           && coherency_line_size().has_value()    //
           && shared_cpu_map().has_value();
  }

  auto to_string() const {
    return fmt::format(
        R"({{"physical_line_partition": {}, "number_of_sets": {}, )"
        R"(ways_of_associativity": {}, "id": {}, "type": {}, "size": {}, )"
        R"("level": {}, "coherency_line_size": {}, "shared_cpu_map": {}}})",
        physical_line_partition().value_or(-1), number_of_sets().value_or(-1),
        ways_of_associativity().value_or(-1), id().value_or(-1),
        type().value_or(-1), size().value_or(-1), level().value_or(-1),
        coherency_line_size().value_or(-1), shared_cpu_map().value_or(-1));
  }

 private:
  unsigned cpu_;
  unsigned index_;

  /**
   * The internal map holding the valid keys and values
   */
  map_type cache_params_{
      {"physical_line_partition", {}},
      {"number_of_sets", {}},
      {"ways_of_associativity", {}},
      {"id", {}},
      {"type", {}},
      {"size", {}},
      {"level", {}},
      {"coherency_line_size", {}},
      {"shared_cpu_map", {}},
  };
};

/**
 * @brief      Cache info for a specific CPU
 */
struct CPUCacheInfo {
  /**
   * No default constructor.
   */
  CPUCacheInfo() = delete;

  /**
   * @brief      Initialise CPUCacheInfo with CacheInfo objects
   *
   * @param[in]  cpu            The cpu
   * @param[in]  infos_or_maps  The infos or maps
   *
   * @tparam     C              The iterable container holding the infos or maps
   * @tparam     <unnamed>      Template helper
   */
  template <typename C,
            typename = std::enable_if_t<
                (exot::utilities::is_iterable_v<C> &&
                 (std::is_same_v<CacheInfo, typename C::value_type> ||
                  std::is_same_v<typename CacheInfo::map_type,
                                 typename C::value_type>))>>
  explicit CPUCacheInfo(unsigned cpu, const C& infos_or_maps) : cpu_{cpu} {
    if constexpr (std::is_same_v<typename C::value_type, CacheInfo>) {
      for (const auto& info : infos_or_maps) {
        caches_.push_back(info);
        caches_map_.insert({{info.level().value(), info.type().value()}, info});
      }
    } else {
      unsigned i{0};
      for (const auto& map : infos_or_maps) {
        auto info = CacheInfo(cpu_, i, map);

        if (!info.is_valid()) {
          throw std::logic_error(
              "CacheInfo for cpu {} created with an CacheInfo or a mapping is "
              "not valid.");
        }

        caches_.push_back(info);
        caches_map_.insert({{info.level().value(), info.type().value()}, info});
        ++i;
      }
    }
  }

  /**
   * @brief      Get info of all caches for a given CPU
   *
   * @param[in]  cpu   The cpu
   */
  explicit CPUCacheInfo(unsigned cpu) : cpu_{cpu} {
    if (cpu > std::thread::hardware_concurrency()) {
      throw std::logic_error(fmt::format("cpu () > hw. concurrency ()", cpu,
                                         std::thread::hardware_concurrency()));
    }

    root_path_ = fmt::format("/sys/devices/system/cpu/cpu{}/cache", cpu);
    /* The root path must be a valid directory. If it's not, it's either not
     * accessible, or does not exist (like in the case of ARM processors). */
    if (!is_directory(root_path_)) {
      throw std::logic_error(fmt::format(
          "the root path is not available/accessible: {}", root_path_));
    }

    for (auto i = int{0};; ++i) {
      if (!is_directory(fmt::format("{}/index{}", root_path_, i))) {
        cache_count_ = i;
        break;
      }
    }

    for (auto i = int{0}; i < cache_count_; ++i) {
      auto _ = CacheInfo(cpu, i);

      if (!_.is_valid()) {
        throw std::logic_error(
            "CacheInfo for cpu {} created from sysfs is not valid.");
      }

      caches_.push_back(_);
      caches_map_.insert({{_.level().value(), _.type().value()}, _});
    }
  }

  auto cpu() const -> unsigned { return cpu_; }
  auto count() const -> unsigned { return cache_count_; }
  const auto& at(size_t idx) const { return caches_.at(idx); }
  const auto& at(int level, int type) const {
    return caches_map_.at({level, type});
  }
  const auto& llc() const { return caches_map_.at({levels(), 2}); }
  const auto& caches() const { return caches_; }
  const auto& caches_map() const { return caches_map_; }
  auto levels() const -> int {
    auto _ = std::vector<int>{};
    for (auto& cache : caches_) {
      if (cache.level().has_value()) { _.push_back(cache.level().value()); }
    }

    return *std::max_element(_.begin(), _.end());
  }

 private:
  unsigned cpu_;
  std::string root_path_;
  unsigned cache_count_{0};
  std::vector<CacheInfo> caches_;
  std::map<std::pair<int, int>, CacheInfo> caches_map_;
};

/**
 * @brief      Decomposes a physical address to tag, set, and offset.
 */
struct AddressDecomposer {
 private:
  unsigned line_size, number_of_sets;
  unsigned long long o, s;
  std::pair<unsigned long long, unsigned long long> olim, slim, tlim;

  void set_limits(const CacheInfo& info) {
    /* Will throw `std::bad_optional_access` if no value is available in cache
     * info. */
    line_size      = info.coherency_line_size().value();
    number_of_sets = info.number_of_sets().value();
    o              = static_cast<unsigned long long>(std::log2(line_size));
    s              = static_cast<unsigned long long>(std::log2(number_of_sets));

    olim = std::make_pair(0ull, o - 1ull);
    slim = std::make_pair(o, o + s - 1ull);
    tlim = std::make_pair(o + s, 63ull);
  }

 public:
  /**
   * @brief      Default constructor, uses LLC address.
   */
  AddressDecomposer() : AddressDecomposer(3u, CacheType::Unified) {}

  AddressDecomposer(const CacheInfo& info) { set_limits(info); }

  /**
   * @brief      Constructs the decomposer for a given cache level (2 or 3)
   *
   * @param[in]  level  The cache level
   */
  explicit AddressDecomposer(unsigned level, CacheType type,
                             std::optional<CPUCacheInfo> info = std::nullopt) {
    if (level > 3 || level < 1) throw std::out_of_range("only L1, L2 and L3");

    /* This assumes that all cores share same cache characteristics (in terms of
     * sizes, ways, etc., not sharing). If a (level, type) pair is not found, a
     * key error will be thrown. */
    auto cache_info = info.has_value() ? info.value() : CPUCacheInfo(0);
    auto cache      = cache_info.at(level, static_cast<int>(type));

    set_limits(cache);
  }

  /**
   * @brief      Get field lengths (tag, set, offset)
   * @note       The length of the tag field will likely not correspond to the
   *             actual length, which is architecture-specific. For example,
   *             on x86_64 processors the physical address can be 48 bits, even
   *             though the ISA allows the entire 64 bit value to be used.
   *             The calculation below assumes a 64 bit address.
   *
   * @return     The field lengths as a tuple.
   */
  auto get_field_lengths() { return std::make_tuple(64ull - (o + s), s, o); }

  /**
   * @brief      Decomposes a physical address to (tag, set, offset)
   *
   * @param[in]  physical_address  The physical address
   *
   * @return     The decomposed address (tag, set, offset)
   */
  auto operator()(unsigned long long physical_address)
      -> std::array<unsigned long long, 3> {
    using namespace exot::utilities;

    return {extract_bit_range(physical_address, tlim.first, tlim.second),
            extract_bit_range(physical_address, slim.first, slim.second),
            extract_bit_range(physical_address, olim.first, olim.second)};
  }

  /**
   * @brief      Gets the tag.
   *
   * @param[in]  physical_address  The physical address
   *
   * @return     The tag.
   */
  inline auto get_tag(unsigned long long physical_address) {
    return extract_bit_range(physical_address, tlim.first, tlim.second);
  }

  /**
   * @brief      Gets the set.
   *
   * @param[in]  physical_address  The physical address
   *
   * @return     The set.
   */
  inline auto get_set(unsigned long long physical_address) {
    return extract_bit_range(physical_address, slim.first, slim.second);
  }

  /**
   * @brief      Gets the offset.
   *
   * @param[in]  physical_address  The physical address
   *
   * @return     The offset.
   */
  inline auto get_offset(unsigned long long physical_address) {
    return extract_bit_range(physical_address, olim.first, olim.second);
  }
};

}  // namespace exot::utilities
