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
 * @file components/function_nodes.h
 * @author     Bruno Klopott
 * @brief      Convienient process network nodes wrapping a single callable
 *             object.
 */

#pragma once

#include <chrono>  // for durations

#include <exot/framework/all.h>  // for framework support

namespace exot::components {

/**
 * All function_* nodes repeatedly run a function consuming and/or producing a
 * token, until the global state is stopped.
 *
 * Functions use `try_{read,write}_for` interface methods to protect against
 * potential deadlocks.
 */

using namespace exot;

/**
 * @brief      A convienience class for declaring nodes that repeatedly run a
 *             function consuming a token
 *
 * @tparam     InputToken  Input token type
 */
template <typename InputToken>
class function_consumer : public framework::IProcess,
                          public framework::Consumer<InputToken> {
 public:
  using node_type  = framework::Consumer<InputToken>;
  using token_type = typename node_type::interface_type::value_type;
  using work_type  = std::function<void(token_type)>;

  using node_type::in_;

  /**
   * The constructor takes a callable typy with signature void(token).
   */
  function_consumer() = delete;
  function_consumer(work_type&& work) : work_{std::forward<work_type>(work)} {};

  void process() override {
    token_type token;

    while (!global_state_->is_stopped()) {
      if (in_.try_read_for(token, std::chrono::seconds{1})) { work_(token); }
    }
  }

 private:
  work_type work_;
};

/**
 * @brief      A convienience class for declaring nodes that repeatedly run a
 *             function producing a token
 *
 * @tparam     Output  Output token type
 */
template <typename OutputToken>
class function_producer : public framework::IProcess,
                          public framework::Producer<OutputToken> {
 public:
  using node_type  = framework::Producer<OutputToken>;
  using token_type = typename node_type::interface_type::value_type;
  using work_type  = std::function<token_type(void)>;

  using node_type::out_;

  /**
   * The constructor takes a callable typy with signature token(void).
   */
  function_producer() = delete;
  function_producer(work_type&& work) : work_{std::forward<work_type>(work)} {};

  void process() override {
    token_type token;

    while (!global_state_->is_stopped()) {
      token = work_();
      while (!out_.try_write_for(token, std::chrono::seconds{1}) &&
             !global_state_->is_stopped()) {};
    }
  }

 private:
  work_type work_;
};

/**
 * @brief      A convienience class for declaring nodes that repeatedly run a
 *             function that first consumes a token, and then produces a token
 *
 * @tparam     InputToken   Input token type
 * @tparam     OutputToken  Output token type
 */
template <typename InputToken, typename OutputToken>
class function_processor
    : public framework::IProcess,
      public framework::Processor<InputToken, OutputToken> {
 public:
  using node_type = framework::Processor<InputToken, OutputToken>;
  using input_token_type =
      typename node_type::consumer_type::interface_type::value_type;
  using output_token_type =
      typename node_type::producer_type::interface_type::value_type;
  using work_type = std::function<output_token_type(input_token_type)>;

  using node_type::in_;
  using node_type::out_;

  /**
   * The constructor takes a callable typy with signature
   * output_token(input_token).
   */
  function_processor() = delete;
  function_processor(work_type&& work)
      : work_{std::forward<work_type>(work)} {};

  void process() override {
    input_token_type input_token;
    output_token_type output_token;

    while (!global_state_->is_stopped()) {
      if (in_.try_read_for(input_token, std::chrono::seconds{1})) {
        output_token = work_(input_token);
        while (!out_.try_write_for(output_token, std::chrono::seconds{1}) &&
               !global_state_->is_stopped()) {};
      }
    }
  }

 private:
  work_type work_;
};

}  // namespace exot::components
