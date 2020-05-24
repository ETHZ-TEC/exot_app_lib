/**
 * @file framework/interface.h
 * @author     Bruno Klopott
 * @brief      Interfaces abstracting to queues and other transmission media.
 */

#pragma once

#include <chrono>  // for time_point and duration
#include <memory>

namespace exot::framework {

/**
 * Mostly adapter classes.
 */

/**
 * Virtual inheritance avoids copies of the base class object. */

/**
 * @brief      Basic reader interface, defining `read()` and `is_readable()`
 *             pure virtual methods.
 *
 * @tparam     Token  Data type used by the interface
 */
template <typename Token>
struct IReader {
  virtual ~IReader()         = default;
  virtual void read(Token&)  = 0;
  virtual bool is_readable() = 0;
};

/**
 * @brief      Basic reader interface, defining `write()` and `is_writeable()`
 *             pure virtual methods.
 *
 * @tparam     Token  Data type used by the interface
 */
template <typename Token>
struct IWriter {
  virtual ~IWriter()               = default;
  virtual void write(const Token&) = 0;
  virtual void write(Token&&)      = 0;
  virtual bool is_writable()       = 0;
};

/**
 * @brief      Base reader class
 * @details    Defines a shared pointer to the channel/container, a method to
 *             set that pointer, and a few member types, which can be used to
 *             access internal types from outside of the interfaces.
 *
 * @tparam     Token      Data type used by the interface
 * @tparam     Container  Type of channel/container used for communication
 */
template <typename Token, template <typename...> typename Container>
class Reader {
 public:
  using value_type = Token;  //! token type transported via the reader interface
  using container_type =
      Container<value_type>;  //! container used by the reader
  using container_pointer =
      std::shared_ptr<container_type>;  //! shared pointer to the container

  Reader() = default;
  explicit Reader(container_pointer container) : in_container_ptr_(container){};

  /**
   * @brief      Set the pointer to the reader's container/channel
   *
   * @param[in]  container  The pointer to the container
   */
  void set_read_container(container_pointer container) {
    in_container_ptr_ = container;
  }

  /**
   * @brief      Get the pointer to the reader's container/channel
   *
   * @return     The pointer to the container
   */
  container_pointer get_read_container() const { return in_container_ptr_; }

 protected:
  container_pointer in_container_ptr_;  //! internal container pointer,
                                        //! accessible by derived classes
};

/**
 * @brief      Base writer class
 * @details    Defines a shared pointer to the channel/container, a method to
 *             set that pointer, and a few member types, which can be used to
 *             access internal types from outside of the interfaces.
 *
 * @tparam     Token      Data type used by the interface
 * @tparam     Container  Type of channel/container used for communication
 */
template <typename Token, template <typename...> typename Container>
class Writer {
 public:
  using value_type = Token;  //! token type transported via the reader interface
  using container_type =
      Container<value_type>;  //! container used by the reader
  using container_pointer =
      std::shared_ptr<container_type>;  //! shared pointer to the container

  Writer() = default;
  explicit Writer(container_pointer container)
      : out_container_ptr_(container){};

  /**
   * @brief      Set the pointer to the writer's container/channel
   *
   * @param[in]  container  The pointer to the container
   */
  void set_write_container(container_pointer container) {
    out_container_ptr_ = container;
  }

  /**
   * @brief      Get the pointer to the writer's container/channel
   *
   * @return     The pointer to the container
   */
  container_pointer get_write_container() const { return out_container_ptr_; }

 protected:
  container_pointer out_container_ptr_;
};

/**
 * @brief      Reader class for accessing queue-like structures.
 */
template <typename Token, template <typename...> typename Container>
class QueueReader : public IReader<Token>, public Reader<Token, Container> {
 public:
  /**
   * This states that the class uses the constructor of the inherited class.
   */
  using Reader<Token, Container>::Reader;
  using Reader<Token,
               Container>::in_container_ptr_;  //! Necessary for name lookup

  /**
   * @brief      Get the size of the underlying container
   * @details    In addition to the basic operations, the queue interface also
   *             provides methods to access the number of elements in the
   *             container.
   *
   * @return     The size of the container
   */
  inline unsigned long size() { return in_container_ptr_->size(); }

  /**
   * @brief      Read and pop a token from the queue
   * @details    Each read operation is destructive, the interface does not
   *             allow peeking into the queue.
   *
   * @param      token  The popped token
   */
  inline void read(Token& token) override {
    token = in_container_ptr_->front();
    in_container_ptr_->pop();
  }

  /**
   * @brief      Determines if the queue is readable
   * @details    The basic queue only provides methods to check if it is empty.
   *             A non-empty queue is considered readable, i.e. a read operation
   *             will return a value.
   *
   * @return     True if readable, False otherwise.
   */
  inline bool is_readable() override { return !in_container_ptr_->empty(); }
};

/**
 * @brief      Writer class for accessing queue-like structures.
 */
template <typename Token, template <typename...> typename Container>
class QueueWriter : public IWriter<Token>, public Writer<Token, Container> {
 public:
  /**
   * This states that the class uses the constructor of the inherited class.
   */
  using Writer<Token, Container>::Writer;
  using Writer<Token, Container>::out_container_ptr_;

  /**
   * @brief      Get the size of the underlying container
   *
   * @return     The size of the container
   */
  inline unsigned long size() { return out_container_ptr_->size(); }

  /**
   * @brief      Write a token to the queue via its const reference
   *
   * @param[in]  token  The token to write to the queue
   */
  inline void write(const Token& token) override {
    out_container_ptr_->push(token);
  }

  /**
   * @brief      Write a token to the queue via a move operation
   * @details    The function only perfect-forwards its argument, the call to
   *             `std::move` must be performed in the calling code.
   *
   * @param[in]  token      The token to write to the queue
   */
  inline void write(Token&& token) override {
    out_container_ptr_->push(std::forward<decltype(token)>(token));
  }

  /**
   * @brief      Determines if the underlying container is writable.
   * @details    The basic queue is not bounded, therefore the call to this
   *             function will always return true.
   *
   * @return     True
   */
  inline bool is_writable() override { return true; }
};

/**
 * @brief      Reader class for accessing queue-like structures that provide
 *             additional functionality of `try_pop`, `try_pop_for`, etc.
 */
template <typename Token, template <typename...> typename Container>
class ExtendedQueueReader : public QueueReader<Token, Container> {
 public:
  /**
   * This states that the class uses the constructor of the inherited class.
   */
  using QueueReader<Token, Container>::QueueReader;
  using QueueReader<Token, Container>::in_container_ptr_;

  /**
   * @brief      Forwarding wrapper for the `try_pop` method.
   */
  inline bool try_read(Token& token) {
    return in_container_ptr_->try_pop(std::forward<decltype(token)>(token));
  };

  /**
   * @brief      Forwarding wrapper for the `try_pop_for` method.
   */
  template <typename Rep, typename Period>
  inline bool try_read_for(Token& token,
                           const std::chrono::duration<Rep, Period>& timeout) {
    return in_container_ptr_->try_pop_for(
        std::forward<decltype(token)>(token),
        std::forward<decltype(timeout)>(timeout));
  };

  /**
   * @brief      Forwarding wrapper for the `try_pop_until` method.
   */
  template <typename Clock, typename Duration>
  inline bool try_read_until(
      Token& token, const std::chrono::time_point<Clock, Duration>& time) {
    return in_container_ptr_->try_pop_until(
        std::forward<decltype(token)>(token),
        std::forward<decltype(time)>(time));
  };
};

/**
 * @brief      Reader class for accessing queue-like structures that provide
 *             additional functionality of `try_push`, `try_push_for`, etc.
 */
template <typename Token, template <typename...> typename Container>
class ExtendedQueueWriter : public QueueWriter<Token, Container> {
 public:
  /**
   * This states that the class uses the constructor of the inherited class.
   */
  using QueueWriter<Token, Container>::QueueWriter;
  using QueueWriter<Token, Container>::out_container_ptr_;

  /**
   * @brief      Forwarding wrapper for the `try_push` method.
   */
  inline bool try_write(const Token& token) {
    return out_container_ptr_->try_push(std::forward<decltype(token)>(token));
  };

  /**
   * @brief      Forwarding wrapper for the `try_push` method.
   */
  inline bool try_write(Token&& token) {
    return out_container_ptr_->try_push(std::forward<decltype(token)>(token));
  };

  /**
   * @brief      Forwarding wrapper for the `try_push_for` method.
   */
  template <typename Rep, typename Period>
  inline bool try_write_for(const Token& token,
                            const std::chrono::duration<Rep, Period>& timeout) {
    return out_container_ptr_->try_push_for(
        std::forward<decltype(token)>(token),
        std::forward<decltype(timeout)>(timeout));
  };

  /**
   * @brief      Forwarding wrapper for the `try_push_for` method.
   */
  template <typename Rep, typename Period>
  inline bool try_write_for(Token&& token,
                            const std::chrono::duration<Rep, Period>& timeout) {
    return out_container_ptr_->try_push_for(
        std::forward<decltype(token)>(token),
        std::forward<decltype(timeout)>(timeout));
  };

  /**
   * @brief      Forwarding wrapper for the `try_push_until` method.
   */
  template <typename Clock, typename Duration>
  inline bool try_write_until(
      const Token& token,
      const std::chrono::time_point<Clock, Duration>& time) {
    return out_container_ptr_->try_push_until(
        std::forward<decltype(token)>(token),
        std::forward<decltype(time)>(time));
  };

  /**
   * @brief      Forwarding wrapper for the `try_push_until` method.
   */
  template <typename Clock, typename Duration>
  inline bool try_write_until(
      Token&& token, const std::chrono::time_point<Clock, Duration>& time) {
    return out_container_ptr_->try_push_until(
        std::forward<decltype(token)>(token),
        std::forward<decltype(time)>(time));
  };

  /**
   * @brief      Determines if the underlying container is writable.
   * @details    The extended queue can be bounded, and provides a `full()`
   *             method.
   *
   * @return     True if writeable, false otherwise.
   */
  inline bool is_writable() override { return out_container_ptr_->full(); }
};

}  // namespace exot::framework
