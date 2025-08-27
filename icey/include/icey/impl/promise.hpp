/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

#pragma once

#include <coroutine>
#include <functional>
#include <icey/impl/result.hpp>
#include <iostream>

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
  using promise_type = Self;
  using Cancel = std::function<void(Self &)>;

  PromiseBase() {
    if (icey_coro_debug_print)
      std::cout << "Promise was default-constructed: " << get_type(*this) << std::endl;
  }
  /// Construct using a handler: This handler is called immediately in the constructor with the
  /// address to this Promise so that it can store it and write to this promise later. This handler
  /// returns a cancellation function that gets called when this Promise is destructed. This
  /// constructor is useful for wrapping an existing callback-based API.
  explicit PromiseBase(std::function<Cancel(Self &)> &&h) {
    if (icey_coro_debug_print) std::cout << "Promise(h) @  " << get_type(*this) << std::endl;
    cancel_ = h(*this);
  }

  PromiseBase(const PromiseBase &) = delete;
  PromiseBase(PromiseBase &&) = delete;
  PromiseBase &operator=(const Self &) = delete;
  PromiseBase &operator=(Self &&) = delete;

  ~PromiseBase() {
    if (icey_coro_debug_print)
      std::cout << "Promise was destructed: " << get_type(*this) << std::endl;
    if (cancel_) cancel_(*this);
  }

  std::suspend_never initial_suspend() { return {}; }
  std::suspend_always final_suspend() const noexcept { return {}; }

  /// Store the unhandled exception in case it occurs: We will re-throw it when it's time. (The
  /// compiler can't do this for us because of reasons)
  void unhandled_exception() { exception_ptr_ = std::current_exception(); }

  /// Await the promise
  auto operator co_await() {
    struct Awaiter {
      Self &promise_;
      Awaiter(Self &promise) : promise_(promise) {}
      bool await_ready() const noexcept {
        if (icey_coro_debug_print)
          std::cout << "Await ready on Promise " << get_type(promise_) << " called" << std::endl;
        return !promise_.has_none();
      }
      void await_suspend(std::coroutine_handle<> continuation) noexcept {
        /// Resume the coroutine when this promise is done
        if (icey_coro_debug_print)
          std::cout << "Await suspend was called, held Promise: " << get_type(promise_)
                    << std::endl;
        //// TODO
        // if(promise_.continuation_) throw std::logic_error("Illegal!");
        promise_.continuation_ = continuation;
      }

      auto await_resume() const noexcept {
        if (icey_coro_debug_print)
          std::cout << "Await resume was called, held Promise: " << get_type(promise_) << std::endl;
        if (promise_.exception_ptr_) std::rethrow_exception(promise_.exception_ptr_);
        return promise_.get_state().get();
      }
    };
    return Awaiter{*this};
  }

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

  void put_value(const Value &value) {
    set_value(value);
    notify();
  }

  void put_error(const Error &error) {
    set_error(error);
    notify();
  }

  void notify() {
    if (continuation_) continuation_.resume();
  }

  State state_;

  /// A synchronous cancellation function. It unregisters for example a ROS callback so that it is
  /// not going to be called anymore. Such cancellations are needed because the ROS callback
  /// captures the Promises object by reference since it needs to write the result to it. But if we
  /// would destruct the Promise and after that this ROS callback gets called, we get an
  /// use-after-free bug. The promise must therefore cancel the ROS callback in the destructor.
  /// Since we have no concurrency, this cancellation function can always be a synchronous function
  /// and thus simply be called in the destructor.
  Cancel cancel_;

  std::coroutine_handle<> continuation_{nullptr};
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
  using State = Base::State;
  using Base::Base;
  using promise_type = Promise<_Value, _Error>;
  promise_type get_return_object() { return {}; }

  auto return_value() { return this->value(); }
  /// return_value (aka. operator co_return) *sets* the value if called with an argument,
  /// very confusing, I know
  /// (Reference: https://devblogs.microsoft.com/oldnewthing/20210330-00/?p=105019)
  void return_value(const _Value &x) {
    if (icey_coro_debug_print) std::cout << get_type(*this) << " setting value " << std::endl;
    this->put_value(x);
  }

  /// Sets the state to the given one:
  void return_value(const State &x) {
    if (icey_coro_debug_print) std::cout << get_type(*this) << " setting state " << std::endl;
    this->put_state(x);
  }
};

/// Specialization so that Promise<void> works. (Nothing interesting here, just C++ 20 coroutine
/// specification being weird)
template <>
class Promise<void, Nothing> : public PromiseBase<Nothing, Nothing> {
public:
  using PromiseBase<Nothing, Nothing>::PromiseBase;
  using promise_type = Promise<void, Nothing>;
  promise_type get_return_object() { return {}; }
  auto return_void() { this->put_value(Nothing{}); }
};

}  // namespace icey