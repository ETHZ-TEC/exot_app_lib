/**
 * @file framework/connect.h
 * @author     Bruno Klopott
 * @brief      Facilities for connecting process network nodes together.
 */

#pragma once

#include <stdexcept>    // for throwing
#include <tuple>        // for packing into tuples and forward_as_tuple
#include <type_traits>  // for is_base_of_v

#include <exot/framework/node.h>     // for node types
#include <exot/utilities/helpers.h>  // for const_for
#include <exot/utilities/types.h>    // for node type traits

namespace exot::framework {

/**
 * @brief      Functor that connects compatible nodes together
 * @details    The simple connect functions were factored into the Connector
 *             functor to allow defining the desired capacity of the created
 *             container interfaces.
 *
 *             If capacity of 0 is used, the default container constructor is
 *             used.
 */
class Connector {
 private:
  const size_t capacity_;

 public:
  /**
   * @brief      Constructs the object with desired capacity.
   *
   * @param[in]  capacity  The deriser capacity
   */
  constexpr Connector(size_t capacity) : capacity_{capacity} {};
  /**
   * @brief      Constructs the object with the default capacity.
   */
  constexpr Connector() : Connector{0} {};

  /**
   * @brief      Connects a valid set of nodes in a pipeline
   * @details    { detailed_item_description }
   *
   * @param[in]  nodes      The nodes to connect together
   *
   * @tparam     Nodes      The types of nodes
   */
  template <typename... Nodes>
  void pipeline(Nodes&&... nodes) {
    /**
     * It does not make sense to connect less than 2 nodes.
     */
    static_assert(sizeof...(nodes) >= 2,
                  "At least two nodes are required to make a pipeline");

    /* Make sure, at compile time, that all node objects inherit from base node
     * class. */
    static_assert(
        (std::is_base_of_v<exot::framework::Node, std::decay_t<Nodes>> && ...),
        "All arguments must be compatible nodes.");

    /* Since it is not possible to simply iterate over a template parameter
     * pack, for convienience a tuple is created, and then accessed through the
     * const-for-loop. */
    auto tuple = std::forward_as_tuple(std::forward<Nodes>(nodes)...);

    /* A valid pipeline has to have a producer and a consumer at the opposite
     * edges. */
    static_assert(exot::utilities::is_producer_v<decltype(std::get<0>(tuple))>,
                  "First node has to be a producer.");
    static_assert(exot::utilities::is_consumer_v<decltype(
                      std::get<sizeof...(nodes) - 1>(tuple))>,
                  "Last node has to be a consumer.");

    /* 'Loop' through nodes and call the pair-wise `connect()` function. */
    exot::utilities::const_for<0, sizeof...(nodes) - 1>([&](auto I) {
      connect_((std::get<I>(tuple)), (std::get<I + 1>(tuple)));
    });
  }

  /**
   * @brief      Perfect forwarding wrapper for internal `connect_` function
   */
  template <typename Left, typename Right>
  void connect(Left&& left, Right&& right) {
    connect_(std::forward<Left>(left), std::forward<Right>(right));
  }

  /**
   * @brief      A variadic connector wrapper, which doesn't validate the
   *             pipeline
   *
   * @param      left   The first node to connect
   * @param      right  The second node to connect
   * @param      rest   The other nodes
   *
   * @tparam     Left   The type of the first node
   * @tparam     Right  The type of the second node
   * @tparam     Rest   The types of the other nodes
   */
  template <typename Left, typename Right, typename... Rest>
  void connect(Left&& left, Right&& right, Rest&&... rest) {
    connect_(std::forward<Left>(left), right);
    connect(std::forward<Right>(right), std::forward<Rest>(rest)...);
  }

 private:
  /**
   * @brief      Connects two compatible nodes together.
   * @details    Also serves as the base case for the recursive-like call to
   *             `connect` through the other template function.
   *
   * @param      left   The node providing an output interface
   * @param      right  The node providing an input interface
   *
   * @tparam     Left   The type of the left node
   * @tparam     Right  The type of the right node
   */
  template <typename Left, typename Right>
  void connect_(Left& left, Right& right) {
    static_assert(exot::utilities::has_output_interface_v<Left>,
                  "Left operand needs to have an output interface defined");

    static_assert(exot::utilities::has_input_interface_v<Right>,
                  "Right operand needs to have an input interface defined");

    static_assert(
        std::is_same_v<
            typename Left::producer_type::interface_type::value_type,
            typename Right::consumer_type::interface_type::value_type>,
        "The value types of Left and Right nodes have to match");

    using container_type =
        typename Left::producer_type::interface_type::container_type;

    static_assert(
        std::is_same_v<
            container_type,
            typename Right::consumer_type::interface_type::container_type>,
        "The container types of Left and Right nodes have to match");

    std::shared_ptr<container_type> container_pointer;

    /* Create the communication interface/container, but do not keep the
     * original pointer/reference. */
    if (capacity_ != 0) {
      container_pointer = std::make_shared<container_type>(capacity_);
    } else {
      container_pointer = std::make_shared<container_type>();
    }

    /* Set output and input containers for left and right nodes. */
    left.set_output(container_pointer);
    right.set_input(container_pointer);

    /* Make sure that both interfaces can access the same container. */
    if (left.get_output() != right.get_input())
      throw std::logic_error(
          "Output and input interfaces point to a different container/channel");
  };
};

namespace details {
template <typename Left, typename Right>
void connect(Left& left, Right& right) {
  static_assert(exot::utilities::has_output_interface_v<Left>,
                "Left operand needs to have an output interface defined");

  static_assert(exot::utilities::has_input_interface_v<Right>,
                "Right operand needs to have an input interface defined");

  static_assert(
      std::is_same_v<typename Left::producer_type::interface_type::value_type,
                     typename Right::consumer_type::interface_type::value_type>,
      "The value types of Left and Right nodes have to match");

  using container_type =
      typename Left::producer_type::interface_type::contaitner_type;

  static_assert(
      std::is_same_v<
          container_type,
          typename Right::consumer_type::interface_type::container_type>,
      "The container types of Left and Right nodes have to match");

  /* Create the communication interface/container, but do not keep the original
   * pointer/reference. */
  auto container_pointer = std::make_shared<container_type>();

  /* Set output and input containers for left and right nodes. */
  left.set_output(container_pointer);
  right.set_input(container_pointer);

  /* Make sure that both interfaces can access the same container. */
  if (left.get_output() != right.get_input())
    throw std::logic_error(
        "Output and input interfaces point to a different container/channel");
};

}  // namespace details

/**
 * @brief      A free-standing variant of the node-connecting function
 * @details    Performs basic validation of passed arguments, makes sure that
 *             the left node has an output, and the right node has an input, an
 *             that they use the same value types and interface container types.
 *
 *             This free-standing function `connect` uses the default container
 *             size.
 *
 * @param      left   The left node
 * @param      right  The right node
 *
 * @tparam     Left   The type of the left node
 * @tparam     Right  The type of the right node
 */
template <typename Left, typename Right>
void connect(Left&& left, Right&& right) {
  details::connect(std::forward<Left>(left), std::forward<Right>(right));
}

/**
 * @brief      A variadic function wrapper to create a sequence of calls to
 *             `connect` via template parameter pack expansion
 *
 * @param      left   The first node to connect
 * @param      right  The second node to connect
 * @param      rest   The other nodes
 *
 * @tparam     Left   The type of the first node
 * @tparam     Right  The type of the second node
 * @tparam     Rest   The types of the other nodes
 */
template <typename Left, typename Right, typename... Rest>
void connect(Left&& left, Right&& right, Rest&&... rest) {
  connect(std::forward<Left>(left), right);
  connect(std::forward<Right>(right), std::forward<Rest>(rest)...);
};

}  // namespace exot::framework
