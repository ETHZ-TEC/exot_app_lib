/**
 * @file primitives/msr.h
 * @author     Bruno Klopott
 * @brief      Module Specific Register accessor class, requires msr kernel
 *             module installed and loaded.
 */

#pragma once

/**
 * Reading the MSRs via the file descriptors works only on Linux-based
 * platforms, excluding Android.
 *
 * @todo On FreeBSD, `cpuctl` can be used with the associated special
 * pseudo-device (and the userspace control utility `cpucontrol`) to read/write
 * machine specific registers. Implementation should be easier than in the case
 * of Linux.
 *
 * On Darwin, there is no straightforward way of accessing the registers,
 * certainly not without developing a kernel extension.
 *
 * On some ARM architectures the performance monitoring unit could be
 * potentially used.
 */
#if defined(__x86_64__) && defined(__linux__) && !defined(__ANDROID__)

#include <fcntl.h>   // for open
#include <unistd.h>  // for pread, prwite

#include <algorithm>         // for all_of, sort, unique
#include <cstdint>           // for uint*
#include <exception>         // for logic_error
#include <fstream>           // for fstream
#include <initializer_list>  // for initializer_list
#include <ios>               // for seekdir
#include <map>               // for mapping between cpu and array index
#include <numeric>           // for iota
#include <string>            // for std::string
#include <thread>            // for hardware_concurrency
#include <vector>            // for variable-size arrays

#include <fmt/format.h>  // for string formatting

#include <exot/utilities/bits.h>  // for bit manipulation

namespace exot::primitives {

class MSR {
 public:
  /**
   * @brief      Constructs the object with an initialiser list
   *
   * @param[in]  cpus  An initialiser list with CPU identifiers
   */
  explicit MSR(std::initializer_list<unsigned> cpus);

  /**
   * @brief      Constructs the object with a vector via const reference
   *
   * @param[in]  cpus  A const reference to a vector with CPU identifiers
   */
  explicit MSR(const std::vector<unsigned>& cpus);

  /**
   * @brief      Constructs the object with a vector via move
   * @details    The vector can be provided in-place, inside the call to the
   *             constructor.
   *
   * @param[in]  cpus  A vector with CPU identifiers
   */
  explicit MSR(std::vector<unsigned>&& cpus);

  /**
   * @brief      The default constructor
   * @details    Provides access to all available MSR pseudo-devices.
   */
  MSR();

  /**
   * @brief      Operator to allow reassignment
   * @details    Since the constructors are responsible for opening/closing file
   *             descriptors, a custom assignment operator is needed, to
   *             reinitialise the access to pseudo-devices.
   *
   * @param[in]  other  The other MSR object
   *
   * @return     The "copied" object
   */
  MSR& operator=(const MSR& other);

  ~MSR();

  /**
   * @brief      Reads a value from a CPU's model specific register
   *
   * @param[in]  cpu   The cpu
   * @param[in]  msr   The msr
   *
   * @return     The value of the MSR
   */
  std::uint64_t read(unsigned int cpu, std::uint64_t msr);

  /**
   * @brief      Reads an MSR from the next configured CPU
   * @details    The helper function is given so that the user of the class does
   *             not necessarily need to know how many MSR interfaces are
   *             accessible. After each read operation the next configured CPU
   *             is used.
   *
   * @param[in]  msr   The msr
   *
   * @return     The value of the MSR
   */
  std::uint64_t read_any(std::uint64_t msr);

  /**
   * @brief      Reads an MSR from the first configured CPU
   *
   * @param[in]  msr   The msr
   *
   * @return     The value of the MSR
   */
  std::uint64_t read_first(std::uint64_t msr);

  /**
   * @brief      Writes a value to a CPU's model specific register
   *
   * @param[in]  cpu    The cpu
   * @param[in]  msr    The msr
   * @param[in]  value  The value
   */
  void write(unsigned int cpu, std::uint64_t msr, uint64_t value);

  /**
   * @brief      Writes an MSR for the next configured CPU
   *
   * @param[in]  msr    The msr
   * @param[in]  value  The value
   */
  void write_any(std::uint64_t msr, std::uint64_t value);

  /**
   * @brief      Writes a value to the first configured CPU's MSR
   *
   * @param[in]  msr    The msr
   * @param[in]  value  The value
   */
  void write_first(std::uint64_t msr, std::uint64_t value);

  /**
   * @brief      Reads a value from all CPU's model specific registers
   *
   * @param[in]  msr   The msr
   *
   * @return     A vector holding the read values
   */
  std::vector<std::uint64_t> read_all(std::uint64_t msr);

  /**
   * @brief      Writes a value to all CPU's model specific registers
   *
   * @param[in]  msr    The msr
   * @param[in]  value  The value
   */
  void write_all(std::uint64_t msr, uint64_t value);

  /**
   * @brief      Gets the filenames of used pseudo-devices
   *
   * @return     The filenames.
   */
  std::vector<std::string> get_filenames() const;
  std::vector<int> get_file_descriptors() const;
  std::vector<unsigned int> get_cpus() const;
  int get_cpu_count() const;
  auto get_index_to_cpu_mapping() const;

 private:
  std::vector<unsigned int> cpus_;      //! vector of CPUs to be accessed
  std::vector<std::string> filenames_;  //! vector of filenames
  std::vector<int> file_descriptors_;   //! vector of file descriptors
  std::map<unsigned int, unsigned int>
      cpu_to_index_;  //! map between CPU number and array index

  std::vector<std::uint64_t> values;

  unsigned int used_cpu_mask_{0};  //! convienience variable used for fast
                                   //! checking if chosen cpu is available.

  /**
   * @brief      Initialises access to pseudo-devices and makes sure they are
   *             readable
   */
  inline void init();

  /**
   * @brief      Sorts and removes duplicates from the vector of CPUs
   */
  inline void conform();

  /**
   * @brief      Checks if any of the provided CPUs exceeds thread count
   */
  inline void check() const;
};

}  // namespace exot::primitives

#endif
