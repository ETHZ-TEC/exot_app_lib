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
 * @file framework/state.h
 * @author     Bruno Klopott
 * @brief      State holder and the global state object.
 */

#pragma once

#include <atomic>   // for atomic variables and operations
#include <csignal>  // for intercepting signals
#include <memory>   // for shared pointers

#include <fmt/format.h>

namespace exot::framework {

/**
 * @brief      Holds global (or local) execution state.
 * @details    Atomic operations ensure safe access and order. They also allow
 *             the class objects to be used in signal handlers.
 */
class State : public std::enable_shared_from_this<State> {
 public:
  using state_type    = State;
  using state_pointer = std::shared_ptr<state_type>;

  /**
   * @brief      Set state to started
   */
  inline void start() { started_.store(true); };
  inline bool is_started() { return started_.load(); };

  /**
   * @brief      Set state to stopped
   */
  inline void stop() { stopped_.store(true); };
  inline bool is_stopped() { return stopped_.load(); };

  /**
   * @brief      Stop and terminate
   */
  inline void terminate() {
    stopped_.store(true);
    terminated_.store(true);
  };
  inline bool is_terminated() { return terminated_.load(); };

  /**
   * @brief      Reset to original values
   */
  inline void reset() {
    started_.store(false);
    stopped_.store(false);
    terminated_.store(false);
  }

  constexpr State() = default;
  ~State()          = default;

  /**
   * Cannot be copied.
   */
  State(const State&) = delete;
  State& operator=(const State&) = delete;

  /**
   * @brief      Get a shared pointer to state object.
   *
   * @return     Shared pointer to specific object instance.
   */
  auto get() { return shared_from_this(); };

 private:
  volatile std::atomic<bool> started_{false};
  volatile std::atomic<bool> stopped_{false};
  volatile std::atomic<bool> terminated_{false};
};

/**
 * Global state used for managing execution. Can be used in signal handlers.
 */
static State::state_pointer GLOBAL_STATE{std::make_shared<State>()};

/**
 * @brief      Handlers for Unix signals
 */
static void interrupt_handler(int) {
  GLOBAL_STATE->stop();
}
static void terminate_handler(int) {
  GLOBAL_STATE->terminate();
}
static void start_handler(int) {
  GLOBAL_STATE->start();
}

/**
 * @brief      Convienience wrapper for setting up global state
 */
static void init_global_state_handlers() {
  std::signal(SIGINT, interrupt_handler);
  std::signal(SIGQUIT, terminate_handler);
  std::signal(SIGUSR1, start_handler);
}

}  // namespace exot::framework
