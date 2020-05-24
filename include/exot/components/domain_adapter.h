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
