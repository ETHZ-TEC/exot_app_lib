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
