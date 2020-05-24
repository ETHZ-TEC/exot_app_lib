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
 * @file components/domain_adapter.h
 * @author     Bruno Klopott
 * @brief      Simple node classes to bridge system and userland thread domains.
 */

#pragma once

#ifdef EXOT_USE_FIBERS

#include <chrono>  // for durations

#include <exot/framework/all.h>  // for framework support

using namespace std::chrono_literals;

namespace exot::components {

/**
 * Node classes to bridge the system and userland thread domains.
 */
inline namespace domain_adapters {
template <typename Token>
class FiberToThread : public exot::framework::IProcess,
                      public exot::framework::IProcessor<
                          Token, exot::framework::FiberTimeoutLockingQueue,
                          exot::framework::ExtendedQueueReader, Token,
                          exot::framework::TimeoutLockingQueue,
                          exot::framework::ExtendedQueueWriter> {
 public:
  using node_type =
      exot::framework::Processor<Token,
                                 exot::framework::FiberTimeoutLockingQueue,
                                 exot::framework::ExtendedQueueReader, Token,
                                 exot::framework::TimeoutLockingQueue,
                                 exot::framework::ExtendedQueueWriter>;
  using token_type    = Token;
  using consumer_type = typename node_type::consumer_type;
  using producer_type = typename node_type::producer_type;

  using node_type::in_;
  using node_type::out_;

  FiberToThread(
      typename consumer_type::interface_type::container_pointer input,
      typename producer_type::interface_type::container_pointer output)
      : node_type(input, output){};
  FiberToThread() : node_type(){};

  void process() override {
    while (!global_state_->is_stopped()) {
      token_type token;
      if (in_.try_read_for(token, 200ms)) { out_.write(token); }
    }
  };
};

template <typename Token>
class ThreadToFiber : public exot::framework::IProcess,
                      public exot::framework::IProcessor<
                          Token, exot::framework::TimeoutLockingQueue,
                          exot::framework::ExtendedQueueReader, Token,
                          exot::framework::FiberTimeoutLockingQueue,
                          exot::framework::ExtendedQueueWriter> {
 public:
  using node_type =
      exot::framework::Processor<Token, exot::framework::TimeoutLockingQueue,
                                 exot::framework::ExtendedQueueReader, Token,
                                 exot::framework::FiberTimeoutLockingQueue,
                                 exot::framework::ExtendedQueueWriter>;
  using token_type    = Token;
  using consumer_type = typename node_type::consumer_type;
  using producer_type = typename node_type::producer_type;

  using node_type::in_;
  using node_type::out_;

  ThreadToFiber(
      typename consumer_type::interface_type::container_pointer input,
      typename producer_type::interface_type::container_pointer output)
      : node_type(input, output){};
  ThreadToFiber() : node_type(){};

  void process() override {
    while (!global_state_->is_stopped()) {
      token_type token;
      if (in_.try_read_for(token, 200ms)) { out_.write(token); }
    }
  };
};

}  // namespace domain_adapters

}  // namespace exot::components

#endif
