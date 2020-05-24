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
 * @file primitives/msr.cpp
 * @author     Bruno Klopott
 * @brief      Implementation of the MSR accessor class.
 */

#if defined(__x86_64__) && defined(__linux__) && !defined(__ANDROID__)

#include <cerrno>
#include <cstring>

#include <exot/primitives/msr.h>

using namespace exot::primitives;

MSR::MSR(std::initializer_list<unsigned> cpus) : cpus_{cpus} {
  check();
  conform();
  init();
}

MSR::MSR(const std::vector<unsigned>& cpus) : cpus_{cpus} {
  check();
  conform();
  init();
}

MSR::MSR(std::vector<unsigned>&& cpus) : cpus_{std::move(cpus)} {
  check();
  conform();
  init();
}

MSR::MSR() {
  auto thread_count = std::thread::hardware_concurrency();
  cpus_.resize(thread_count);
  std::iota(cpus_.begin(), cpus_.end(), 0);

  check();
  conform();
  init();
}

MSR& MSR::operator=(const MSR& other) {
  cpus_ = other.cpus_;
  filenames_.erase(filenames_.cbegin(), filenames_.cend());
  file_descriptors_.erase(file_descriptors_.cbegin(), file_descriptors_.cend());
  cpu_to_index_.erase(cpu_to_index_.cbegin(), cpu_to_index_.cend());

  /**
   * Need to reopen the pseudo-devices and update mapping.
   */
  init();

  return *this;
}

MSR::~MSR() {
  for (auto& fd : file_descriptors_) { ::close(fd); }
}

std::uint64_t MSR::read(unsigned int cpu, std::uint64_t msr) {
  /* Although it should be possible to replicate the functionality of `pread`
   * with more idiomatic code (via an fstream opened in binary mode, and
   * fstream::{seekg, peekg, read}), an initial attempt did not achieve the
   * same functionality. Nevertheless, `pread` is said to be thread-safe and
   * should allow concurrent access to a single file descriptor. */

  assert(((1 << cpu) & used_cpu_mask_) != 0);

  std::uint64_t value;

  auto ret = pread(file_descriptors_.at(cpu_to_index_[cpu]),
                   reinterpret_cast<void*>(&value), sizeof(value), msr);

  if (ret == -1) {
    throw std::runtime_error(fmt::format("[MSR] error ({}) in pread: {}", errno,
                                         std::strerror(errno)));
  }

  return value;
}

std::uint64_t MSR::read_any(std::uint64_t msr) {
  static short i{0};
  static short n{static_cast<short>(cpus_.size())};
  i = i == n ? 0 : i;  //! require to prevent overflows
  return read(cpus_.at(i++ % n), msr);
}

std::uint64_t MSR::read_first(std::uint64_t msr) {
  return read(cpus_.at(0), msr);
}

void MSR::write(unsigned int cpu, std::uint64_t msr, uint64_t value) {
  auto ret = pwrite(file_descriptors_.at(cpu_to_index_[cpu]),
                    reinterpret_cast<void*>(&value), sizeof(value), msr);

  if (ret == -1) {
    throw std::runtime_error(fmt::format("[MSR] error ({}) in pwrite: {}",
                                         errno, std::strerror(errno)));
  }
}

void MSR::write_any(std::uint64_t msr, std::uint64_t value) {
  static short i{0};
  static short n{static_cast<short>(cpus_.size())};
  i = i == n ? 0 : i;  //! required to prevent overflows
  write(cpus_.at(i++ % n), msr, value);
}

void MSR::write_first(std::uint64_t msr, std::uint64_t value) {
  write(cpus_.at(0), msr, value);
}

std::vector<std::uint64_t> MSR::read_all(std::uint64_t msr) {
  for (auto cpu : cpus_) { values.at(cpu_to_index_[cpu]) = (read(cpu, msr)); }

  return values;
}

void MSR::write_all(std::uint64_t msr, uint64_t value) {
  for (auto cpu : cpus_) { write(cpu, msr, value); }
}

std::vector<std::string> MSR::get_filenames() const {
  return filenames_;
}
std::vector<int> MSR::get_file_descriptors() const {
  return file_descriptors_;
}
std::vector<unsigned int> MSR::get_cpus() const {
  return cpus_;
}
int MSR::get_cpu_count() const {
  return cpus_.size();
}
auto MSR::get_index_to_cpu_mapping() const {
  return cpu_to_index_;
}

void MSR::init() {
  std::vector<std::fstream> descriptors;  //! local vector of stream objects
                                          //! used for checking access

  unsigned int index{0};

  /* Create filenames and check if descriptors are readable. */
  for (auto cpu : cpus_) {
    auto filename = fmt::format("/dev/cpu/{}/msr", cpu);
    filenames_.push_back(filename);
    descriptors.emplace_back(filename,
                             std::ios::in | std::ios::out | std::ios::binary);

    /* Mapping between cpu and array index. */
    cpu_to_index_[cpu] = index;
    ++index;

    exot::utilities::set_bit(used_cpu_mask_, cpu);
  }

  /* Check if all file descriptors are readable, if not, throw. */
  bool all_readable =
      std::all_of(descriptors.begin(), descriptors.end(),
                  [](const auto& descriptor) { return descriptor.is_open(); });

  if (!all_readable) {
    throw std::logic_error("At least one MSR pseudo-device is not readable.");
  }

  for (auto&& descriptor : descriptors) { descriptor.close(); }

  for (const auto& filename : filenames_) {
    int fd = ::open(filename.c_str(), O_RDWR);
    if (fd < 0)
      throw std::logic_error(
          fmt::format("Opening MSR pseudo-device {} failed.", filename));

    file_descriptors_.push_back(fd);
  }

  values.resize(cpus_.size());
}

void MSR::conform() {
  std::sort(cpus_.begin(), cpus_.end());
  auto last_unique = std::unique(cpus_.begin(), cpus_.end());
  cpus_.erase(last_unique, cpus_.end());
}

void MSR::check() const {
  if (!std::all_of(cpus_.begin(), cpus_.end(), [](auto el) {
        return el < std::thread::hardware_concurrency();
      })) {
    throw std::out_of_range(
        "At least one element in the CPU vector exceeds hardware "
        "concurrency.");
  }
}

#endif
