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
 * @file utilities/synchronised.h
 * @author     Bruno Klopott
 * @brief      Synchronised type wrappers
 */

#pragma once

#include <mutex>
#include <type_traits>

namespace exot::utilities {

/**
 * @brief      Synchronised type wrapper
 *
 * @tparam     T     The wrapped type
 */
template <typename T>
struct synchronised_t {
  using wrapped_type = std::decay_t<T>;
  using lock_type    = std::mutex;
  using guard_type   = std::lock_guard<lock_type>;

  /**
   * @brief      Constructs a synchronised object with const reference
   *
   * @param[in]  object  The object
   */
  synchronised_t(const T& object) : object_{object} {}

  /**
   * @brief      Constructs a synchronised object with a move (ref or value)
   *
   * @param      object  The object
   */
  synchronised_t(T&& object) : object_{std::move(object)} {}

  /**
   * @brief      Constructs a synchronised object by copying
   *
   * @param[in]  other  The other
   */
  synchronised_t(const synchronised_t& other) {
    guard_type{lock_};
    other.access([&](auto& _) { object_ = _; });
  }

  /**
   * @brief      Assigns wrapped contents from another passed by const ref
   *
   * @param[in]  other  The other
   *
   * @return     The reference to the updated object
   */
  synchronised_t& operator=(const synchronised_t& other) {
    guard_type{lock_};
    object_ = other.operator wrapped_type();
    return *this;
  }

  /**
   * @brief      Assigns wrapped contents from another passed by 'mutable' ref
   *
   * @param[in]  other  The other
   *
   * @return     The reference to the updated object
   */
  synchronised_t& operator=(synchronised_t& other) {
    guard_type{lock_};
    other.access([&](auto& _) { object_ = _; });
    return *this;
  }

  /**
   * @brief      Accesses the wrapped contents while holding a lock using a
   *             callable
   *
   * @param      callable  The callable
   *
   * @tparam     Callable  The type of the callable
   *
   * @return     The return value of the callable
   */
  template <typename Callable>
  auto access(Callable&& callable) {
    return guard_type{lock_}, std::forward<Callable>(callable)(object_);
  }

  /* Guarded operators for access to the wrapped object
   *
   * Functions below use the comma operator. When (A, B) is evaluated, the
   * result of A is discarded, and it's side effects are completed before B is
   * evaluated. However, if A has class type, it won't be destroyed until the
   * end of the entire expression. Therefore if used with the lock guard, the
   * lock will be held until the operation is finished.
   */
  wrapped_type* operator->() { return guard_type{lock_}, &object_; }
  wrapped_type* operator&() { return guard_type{lock_}, &object_; }
  wrapped_type& operator*() { return guard_type{lock_}, object_; }
  operator wrapped_type() const { return guard_type{lock_}, object_; }

 private:
  wrapped_type object_;  //! the wrapped object
  lock_type lock_;       //! the guarding lock
};

}  // namespace exot::utilities
