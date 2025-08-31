/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

#pragma once

#include <coroutine>
#include <functional>
#include <icey/impl/result.hpp>
#include <iostream>

#ifdef ICEY_CORO_DEBUG_PRINT
#include <fmt/format.h>

#include <boost/type_index.hpp>
/// Returns a string that represents the type of the given value
template <class T>
static std::string get_type(T &t) {
  return fmt::format("[{} @ 0x{:x}]", boost::typeindex::type_id_runtime(t).pretty_name(),
                     std::size_t(&t));
}
#endif

/// This header defines a promise type supporting async/await (C++ 20 coroutines) only.
/// Not thread-safe.
namespace icey {

template <typename, typename = std::void_t<>>
struct has_promise_type : std::false_type {};

template <typename T>
struct has_promise_type<T, std::void_t<typename T::promise_type>> : std::true_type {};

template <typename T>
inline constexpr bool has_promise_type_v = has_promise_type<T>::value;

inline bool icey_coro_debug_print = false;

template <class Value, class Error>
class task;
struct PromiseTag {};
/// A Promise is an asynchronous abstraction that yields a single value or an error.
/// I can be used with async/await syntax coroutines in C++20.
/// It also allows for wrapping an existing callback-based API.
/// It does not use dynamic memory allocation to store the value.

/// For references, see the promise implementations of this library:
/// [asyncpp] https://github.com/asyncpp/asyncpp/blob/master/include/asyncpp/promise.h
template <class _Value = Nothing, class _Error = Nothing>
class PromiseBase : public PromiseTag {
public:
  using Value = _Value;
  using Error = _Error;
  using State = Result<_Value, _Error>;
  using Self = PromiseBase<Value, Error>;
  using Cancel = std::function<void(Self &)>;

  PromiseBase() {
#ifdef ICEY_CORO_DEBUG_PRINT
    std::cout << get_type(*this) << " Constructor()" << std::endl;
#endif
  }

  PromiseBase(const PromiseBase &) = delete;
  PromiseBase(PromiseBase &&) = delete;
  PromiseBase &operator=(const Self &) = delete;
  PromiseBase &operator=(Self &&) = delete;

  ~PromiseBase() {
#ifdef ICEY_CORO_DEBUG_PRINT
    std::cout << get_type(*this) << " Destructor()" << std::endl;
#endif
    if (cancel_) cancel_(*this);
  }
  friend struct final_awaitable;
  struct final_awaitable {
    auto await_ready() const noexcept -> bool { return false; }

    template <typename promise_type>
    auto await_suspend(std::coroutine_handle<promise_type> coroutine) noexcept
        -> std::coroutine_handle<> {
      auto &promise = coroutine.promise();
#ifdef ICEY_CORO_DEBUG_PRINT
      std::cout
          << "promise_base await_suspend was called on promise, transfering to continuation ..: "
          << get_type(promise) << std::endl;
#endif
      // If there is a continuation call it, otherwise this is the end of the line.
      if (promise.continuation_ != nullptr) {
        return promise.continuation_;
      } else {
        return std::noop_coroutine();
      }
    }

    auto await_resume() noexcept -> void {
      // no-op
    }
  };

  auto initial_suspend() {
    return std::suspend_never{};
    /*
    if constexpr (std::is_same_v<Value, Nothing>)
    return std::suspend_never{};
    else
    return std::suspend_always{};
    */
  }
  auto final_suspend() const noexcept {
    return std::suspend_always{};
    // return final_awaitable{};
  }

  /// Store the unhandled exception in case it occurs: We will re-throw it when it's time. (The
  /// compiler can't do this for us because of reasons)
  void unhandled_exception() { exception_ptr_ = std::current_exception(); }

  bool has_none() const { return state_.has_none(); }
  bool has_value() const { return state_.has_value(); }
  bool has_error() const { return state_.has_error(); }
  const Value &value() const { return state_.value(); }
  const Error &error() const { return state_.error(); }
  State &get_state() { return state_; }
  const State &get_state() const { return state_; }

  /// Sets the state to hold none, but does not notify about this state change.
  void set_none() { state_.set_none(); }
  /// Sets the state to hold a value, but does not notify about this state change.
  void set_value(const Value &x) { state_.set_value(x); }
  /// Sets the state to hold an error, but does not notify about this state change.
  void set_error(const Error &x) { state_.set_error(x); }
  void set_state(const State &x) { state_ = x; }

  void resolve(const Value &value) {
    set_value(value);
    notify();
  }

  void reject(const Error &error) {
    set_error(error);
    notify();
  }

  void put_state(const State &error) {
    set_state(error);
    notify();
  }

  void notify() {
    if (continuation_) continuation_.resume();
  }

  bool was_handle_ctored_{false};

  /// State of the promise: May be nothing, value or error.
  State state_;

  /// A synchronous cancellation function. It unregisters for example a ROS callback so that it is
  /// not going to be called anymore. Such cancellations are needed because the ROS callback
  /// captures the Promises object by reference since it needs to write the result to it. But if we
  /// would destruct the Promise and after that this ROS callback gets called, we get an
  /// use-after-free bug. The promise must therefore cancel the ROS callback in the destructor.
  /// Since we have no concurrency, this cancellation function can always be a synchronous function
  /// and thus simply be called in the destructor.
  Cancel cancel_;

  /// The continuation that is registered when co_awaiting this promise
  std::coroutine_handle<> continuation_{nullptr};
  /// An exception that may be present additionally
  std::exception_ptr exception_ptr_{nullptr};
};

/// A Promise is used in ICEY for async/await. It is returned from asynchronous operations such as a
/// service calls or TF lookups. By calling co_await on a Promise, an icey::Result is returned.
///
/// Dev doc: The whole point of this PromiseBase and Promise is the weird design of the promise
/// interface that is required for C++20 coroutines: The operator co_return that isn't actually
/// called operator co_return consists of two differently named functions, return_value and
/// return_void. Only one of these two functions must be declared, depending on whether a Promise
/// holds a value or not, i.e. is the void-version. If you now think "can't I just choose between
/// return_value and return_void using SFINAE?", nope this does not work (Reference:
/// https://devblogs.microsoft.com/oldnewthing/20210330-00/?p=105019) So we need to use different
/// classes and partial specialization.
template <class _Value, class _Error = Nothing>
class Promise : public PromiseBase<_Value, _Error> {
public:
  using Base = PromiseBase<_Value, _Error>;
  using Cancel = Base::Cancel;
  using State = Base::State;
  using Base::Base;

  task<_Value, _Error> get_return_object();

  // auto return_value() { return this->value(); }

  /// Construct using a handler: This handler is called immediately in the constructor with the
  /// address to this Promise so that it can store it and write to this promise later. This handler
  /// returns a cancellation function that gets called when this Promise is destructed. This
  /// constructor is useful for wrapping an existing callback-based API.
  auto return_value(std::function<void(Base &, Cancel &)> &&h) {
    h(*this, this->cancel_);
#ifdef ICEY_CORO_DEBUG_PRINT
    std::cout << get_type(*this) << " return_value(handle)-" << std::endl;
#endif
    this->was_handle_ctored_ = true;
  }

  /// return_value (aka. operator co_return) *sets* the value if called with an argument,
  /// very confusing, I know
  /// (Reference: https://devblogs.microsoft.com/oldnewthing/20210330-00/?p=105019)
  void return_value(const _Value &x) { this->resolve(x); }

  /// Sets the state to the given one:
  void return_value(const State &x) { this->put_state(x); }
};

/// Specialization so that Promise<void> works. (Nothing interesting here, just C++ 20 coroutine
/// specification being weird)
template <>
class Promise<void, Nothing> : public PromiseBase<Nothing, Nothing> {
public:
  using Base = PromiseBase<Nothing, Nothing>;
  using Cancel = Base::Cancel;

  task<void, Nothing> get_return_object();

  auto return_void() { this->resolve(Nothing{}); }
};

template <class _Value, class _Error = Nothing>
class [[nodiscard]] task {
public:
  using Self = task<_Value, _Error>;
  using Value = _Value;
  using Error = _Error;
  using promise_type = Promise<_Value, _Error>;

  using coroutine_handle = std::coroutine_handle<promise_type>;

  struct awaitable_base {
    awaitable_base(coroutine_handle coroutine) noexcept : m_coroutine(coroutine) {}

    bool await_ready() const noexcept {
      bool is_ready = !m_coroutine || m_coroutine.done();
      auto &p = m_coroutine.promise();
#ifdef ICEY_CORO_DEBUG_PRINT
      std::cout << "task await_ready called on promise: " << get_type(p) << "is ready: " << is_ready
                << std::endl;
#endif
      return false;
      return is_ready;
    }

    auto await_suspend(std::coroutine_handle<> awaiting_coroutine) noexcept {
      auto &p = m_coroutine.promise();
#ifdef ICEY_CORO_DEBUG_PRINT
      std::cout << get_type(p) << " await_suspend(), setting continuation .." << std::endl;
#endif
      p.continuation_ = awaiting_coroutine;
      fmt::print("m_coroutine is: 0x{:x}, continuation_ is: 0x{:x}\n", std::size_t(m_coroutine.address()), 
          std::size_t(p.continuation_.address()));
      return true;
      // return m_coroutine;
      //      return promise_.was_handle_ctored_;
    }
    auto await_resume() {
      auto &promise = this->m_coroutine.promise();
#ifdef ICEY_CORO_DEBUG_PRINT
      std::cout << get_type(promise) << " await_resume()" << std::endl;
#endif
      if (promise.exception_ptr_) std::rethrow_exception(promise.exception_ptr_);
      if constexpr (std::is_same_v<Value, Nothing>)
        return;
      else {
        if (promise.has_none()) {
          throw std::invalid_argument("Promise has nothing, called resume too early.");
        }
        return promise.get_state().get();
      }
    }

    std::coroutine_handle<promise_type> m_coroutine{nullptr};
  };

  task() noexcept : m_coroutine(nullptr) {
#ifdef ICEY_CORO_DEBUG_PRINT
    std::cout << get_type(*this) << " Constructor()" << std::endl;
#endif
  }

  explicit task(coroutine_handle handle) : m_coroutine(handle) {
#ifdef ICEY_CORO_DEBUG_PRINT
    std::cout << get_type(*this) << " Constructor(handle) (get_return_object)" << std::endl;
#endif
  }
  task(const task &) = delete;
  task(task &&other) = delete;
  task &operator=(const task &) = delete;
  task &operator=(task &&other) = delete;

  ~task() {
#ifdef ICEY_CORO_DEBUG_PRINT

    std::cout << get_type(*this) << " Destructor(), m_coroutine.done(): " << m_coroutine.done() << std::endl;
#endif
    if (m_coroutine && m_coroutine.done()) {
      m_coroutine.destroy();
      m_coroutine = nullptr;
    }
    /*
    #ifdef ICEY_CORO_DEBUG_PRINT
        auto& p = m_coroutine.promise();
        std::cout << "~task destroying promise: " << get_type(p) << std::endl;
    #endif
        if (m_coroutine != nullptr) {
          m_coroutine.destroy();
        }*/
  }

  /*auto resume() -> bool {
    if (!m_coroutine.done()) {
      m_coroutine.resume();
    }
    return !m_coroutine.done();
  }

  auto destroy() -> bool {
    if (m_coroutine != nullptr) {
      m_coroutine.destroy();
      m_coroutine = nullptr;
      return true;
    }

    return false;
  }*/

  auto operator co_await() const &noexcept { return awaitable_base{m_coroutine}; }

  auto promise() & -> promise_type & { return m_coroutine.promise(); }
  auto promise() const & -> const promise_type & { return m_coroutine.promise(); }
  auto promise() && -> promise_type && { return std::move(m_coroutine.promise()); }

  auto handle() -> coroutine_handle { return m_coroutine; }

private:
  coroutine_handle m_coroutine{nullptr};
};

template <class Value, class Error>
inline task<Value, Error> Promise<Value, Error>::get_return_object() noexcept {
#ifdef ICEY_CORO_DEBUG_PRINT
  std::cout << get_type(*this) << " get_return_object() " << std::endl;
#endif
  return task<Value, Error>{std::coroutine_handle<Promise<Value, Error>>::from_promise(*this)};
}

inline task<void, Nothing> Promise<void, Nothing>::get_return_object() noexcept {
#ifdef ICEY_CORO_DEBUG_PRINT
  std::cout << get_type(*this) << " get_return_object() " << std::endl;
#endif
  return task<void, Nothing>{std::coroutine_handle<Promise<void, Nothing>>::from_promise(*this)};
}

}  // namespace icey