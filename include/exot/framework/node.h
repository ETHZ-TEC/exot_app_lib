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
 * @file framework/node.h
 * @author     Bruno Klopott
 * @brief      Definitions of the core process network node: processes,
 *             producers, consumers, and processors.
 */

#pragma once

#include <memory>  // for shared_ptr

#include <exot/framework/interface.h>  // for in/out interfaces
#include <exot/framework/state.h>      // for State and GLOBAL_STATE

/**
 * Declare GLOBAL_STATE as extern for linking.
 */
extern std::shared_ptr<exot::framework::State> exot::framework::GLOBAL_STATE;

namespace exot::framework {

/**
 * @brief      Separate process() and task() interfaces such that a node can
 *             implement either or both ivocation styles (loop and/or single
 *             task) */

/**
 * @brief      Class for executable nodes using the continuous process model,
 *             which run until a termination condition is set in the local or
 *             global state.
 */
class IProcess {
 public:
  using state_type    = exot::framework::State;
  using state_pointer = std::shared_ptr<state_type>;

  /**
   * @brief      Constructs the object.
   *
   * @param[in]  global_state  The pointer to a state object
   */
  explicit IProcess(state_pointer global_state)
      : global_state_{global_state} {};

  /**
   * @brief      Constructs the default object.
   * @details    The delegated constructor sets the state pointer to the global
   *             state object's shared pointer
   */
  IProcess() : IProcess(GLOBAL_STATE->get()){};

  /**
   * @brief      Destroys the object.
   * @details    Virtual constructors are necessary for base classes.
   */
  virtual ~IProcess() = default;

  /**
   * @brief      The main activity of the process, has to be implemented in
   *             deriving classes.
   */
  virtual void process() = 0;

 protected:
  state_pointer
      global_state_;  //! A shared pointer to the (most likely) global state
};

/**
 * @todo Currently there is no support for task-based nodes.
 * @brief      Enum class for the return type of a task.
 */
enum class TaskStatus;

/**
 * @todo Currently there is no support for task-based nodes.
 * @brief      Class for executable nodes using the task model, which execute a
 *             single block of code and return, i.e. do not have an internal
 *             processing loop.
 *
 * @tparam     Args  Argument types used for task arguments.
 */
template <typename... Args>
class ITask {
 public:
  virtual ~ITask()                      = default;
  virtual TaskStatus task(Args... args) = 0;
};

/**
 * @brief      Base class for node classes, used possibly for polymorphic
 *             access/storage, or type traits.
 */
struct Node {
  virtual ~Node() = default;
};

/**
 * @brief      Helper base class for determining if node is a consumer in
 *             templates.
 */
struct TConsumer {
  virtual ~TConsumer() = default;
};

/**
 * @brief      Helper base class for determining if node is a producer in
 *             templates.
 */
struct TProducer {
  virtual ~TProducer() = default;
};

/**
 * @brief      Helper base class for determining if node is a processor in
 *             templates.
 */
struct TProcessor {
  virtual ~TProcessor() = default;
};

/**
 * @details    The subsequent classes use template template parameters, which
 *             work as follows: the first template argument is passed to the
 *             second, then second is passed to the third. Thanks to that there
 *             is no need to write e.g. `Consumer<T, Container<T>, Reader<T,
 *             Container<T>>`, instead it is sufficient to write `Consumer<T,
 *             Container, Reader>`. This approach is used in many STL classes.
 *
 *             Virtual inheritance of the base class prevents multiple base
 *             objects being present in classes which derive from both Consumer
 *             and Producer classes.
 */

/**
 * @brief      The consumer node class, which defines an input interface for
 *             specific token type, with a specific communication channel.
 *
 * @tparam     Token      The data type carried via the channel/container
 * @tparam     Container  The type of the communication channel/container
 * @tparam     Reader     The interface type used to access the communication
 *                        channel/container.
 */
template <typename Token, template <typename...> typename Container,
          template <typename, template <typename...> typename> typename Reader>
class IConsumer : public virtual Node, public virtual TConsumer {
 public:
  using consumer_type  = IConsumer<Token, Container, Reader>;
  using interface_type = Reader<Token, Container>;

  IConsumer() = default;
  explicit IConsumer(typename interface_type::container_pointer input_queue)
      : in_(input_queue){};
  virtual ~IConsumer() = default;

  void set_input(typename interface_type::container_pointer input) {
    in_.set_read_container(input);
  }
  typename interface_type::container_pointer get_input() const {
    return in_.get_read_container();
  }

 protected:
  interface_type in_;  //! Input interface, accessible by derived classes.
};

/**
 * @brief      The producer node class, which defines an output interface for
 *             specific token type, with a specific communication channel.
 *
 * @tparam     Token      The data type carried via the channel/container
 * @tparam     Container  The type of the communication channel/container
 * @tparam     Writer     The interface type used to access the communication
 *                        channel/container.
 */
template <typename Token, template <typename...> typename Container,
          template <typename, template <typename...> typename> typename Writer>
class IProducer : public virtual Node, public virtual TProducer {
 public:
  using producer_type  = IProducer<Token, Container, Writer>;
  using interface_type = Writer<Token, Container>;

  IProducer() = default;
  explicit IProducer(typename interface_type::container_pointer output_queue)
      : out_(output_queue){};
  virtual ~IProducer() = default;

  void set_output(typename interface_type::container_pointer output) {
    out_.set_write_container(output);
  }
  typename interface_type::container_pointer get_output() const {
    return out_.get_write_container();
  }

 protected:
  interface_type out_;  //! Output interface, accessible by derived classes.
};

/**
 * @brief      The processor node class, which combines the functionality of a
 *             producer and a consumer, with both input and output interfaces
 *             and distinct token types.
 *
 * @tparam     InputToken       The input data type carried via the input
 *                              channel/container
 * @tparam     InputContainer   The type of the input communication
 *                              channel/container
 * @tparam     Reader           The interface type used to access the input
 *                              communication channel/container.
 * @tparam     OutputToken      The output data type carried via the output
 *                              channel/container
 * @tparam     OutputContainer  The type of the output communication
 *                              channel/container
 * @tparam     Writer           The interface type used to access the
 *                              communication channel/container.
 */
template <typename InputToken, template <typename...> typename InputContainer,
          template <typename, template <typename...> typename> typename Reader,
          typename OutputToken, template <typename...> typename OutputContainer,
          template <typename, template <typename...> typename> typename Writer>
class IProcessor : public IConsumer<InputToken, InputContainer, Reader>,
                   public IProducer<OutputToken, OutputContainer, Writer>,
                   public virtual TProcessor {
 public:
  /**
   * These types are used both internally and externally.
   */
  using consumer_type =
      typename IConsumer<InputToken, InputContainer, Reader>::consumer_type;
  using producer_type =
      typename IProducer<OutputToken, OutputContainer, Writer>::producer_type;
  /* Since the base classes are template classes, this declaration is
   * necessary for easy access to the interface, without the need of stating
   * thebase class every time.*/
  using consumer_type::in_;
  using producer_type::out_;

  IProcessor() = default;
  explicit IProcessor(
      typename consumer_type::interface_type::container_pointer input_queue,
      typename producer_type::interface_type::container_pointer output_queue)
      : consumer_type(input_queue), producer_type(output_queue){};
  virtual ~IProcessor() = default;
};

};  // namespace exot::framework
