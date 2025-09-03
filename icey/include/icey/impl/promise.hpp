/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

#pragma once

#include <coroutine>
#include <functional>
#include <icey/impl/result.hpp>
#include <iostream>
#include <memory>
#ifdef ICEY_CORO_DEBUG_PRINT
#include <fmt/format.h>

#include <boost/type_index.hpp>
#include <thread>  // for ID
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
class Task;
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
  /// This is a function that launches the asynchronous operation. It is used to wrap a
  /// callback-based API. This promise stores this function so that it can be called on
  /// await_suspend. The callback-based API receives this promise and will write the result into
  /// this promise (first argument Self&). It also sets Cancel function (second argument)
  using LaunchAsync = std::function<void(Self &, Cancel &)>;

  PromiseBase(){
#ifdef ICEY_CORO_DEBUG_PRINT
  // std::cout << get_type(*this) << " Constructor()" << std::endl;
// std::cout << get_type(*this) << " Constructor()" << std::endl;
#endif
  }

  PromiseBase(const PromiseBase &) = delete;
  PromiseBase(PromiseBase &&) = delete;
  PromiseBase &operator=(const Self &) = delete;
  PromiseBase &operator=(Self &&) = delete;

  /// calls the cancel function if it was set
  ~PromiseBase() {
#ifdef ICEY_CORO_DEBUG_PRINT
    // std::cout << get_type(*this) << " Destructor()" << std::endl;
    std::cout << fmt::format("Destructing coroutine state: 0x{:x} (Promise {})\n",
                             size_t(std::coroutine_handle<Self>::from_promise(*this).address()),
                             get_type(*this))
              << std::endl;
#endif
    /// cancellation only happens when the promise is destroyed before it has a value: This happens
    /// only when the user forgets to add a co_await
    if (cancel_ && has_none()) cancel_(*this);
  }

  auto initial_suspend() const noexcept {
    struct Awaiter {
      bool await_ready() const noexcept {
        //std::cout << "initial await_ready" << std::endl;
        return false;
      }
      bool await_suspend(std::coroutine_handle<> awaiting_coroutine) const noexcept {
           std::cout << fmt::format("initial await_suspend(), awaited by 0x{:x}\n",
                                   std::size_t(awaiting_coroutine.address()));
        
        return true;
      }
      void await_resume() const noexcept {}
    };
    return Awaiter{};
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

  /// Calls the continuation coroutine
  void notify() {
    if (continuation_) {
      /// If a coro handle is done, the function ptr is nullptr, so we get a crash on resume
      if (!continuation_.done()) {
#ifdef ICEY_CORO_DEBUG_PRINT
        std::cout << fmt::format("Continuing coroutine: 0x{:x}\n", size_t(continuation_.address()))
                  << std::endl;
#endif
        continuation_.resume();
      } else {
#ifdef ICEY_CORO_DEBUG_PRINT
        fmt::print("NOT continuing coroutine: 0x{:x}, it is done!\n",
                   size_t(continuation_.address()));
        getchar();

#endif
      }
    }
  }

  /// Get the result of the promise: Re-throws an exception if any was stored, other gets the state.
  auto get() {
    if (exception_ptr_) {
      std::rethrow_exception(exception_ptr_);
    }
    if constexpr (std::is_same_v<Value, Nothing>)
      return;
    else {
      if (has_none()) {
        std::cout << "Promise has nothing, called resume too early." << std::endl;
        getchar();
      }
      return get_state().get();
    }
  }

  /// Launches the asynchronous operation that was stored previously (if any). Sets the given
  /// continuation.
  void launch_async(std::coroutine_handle<> continuation) {
    continuation_ = continuation;
    if (launch_async_) launch_async_(*this, this->cancel_);
  }

  /// State of the promise: May be nothing, value or error.
  State state_;

  /// Function that starts an asynchronous ROS operation like a service call. Must be asynchronous,
  /// i.e. run in the event-loop, no synchronous operations are allowed (even if the result is
  /// already available).
  LaunchAsync launch_async_;

  /// A synchronous cancellation function. It unregisters for example a ROS callback so that it is
  /// not going to be called anymore. Such cancellations are needed because the ROS callback
  /// captures the Promises object by reference since it needs to write the result to it. But if we
  /// would destruct the Promise and after that this ROS callback gets called, we get an
  /// use-after-free bug. The promise must therefore cancel the ROS callback in the destructor.
  /// Everything that we have in ROS can be cancelled synchronously, so a synchronous cancellation
  /// function called in the destructor is sufficient.
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
  using Self = Promise<_Value, _Error>;
  using Base = PromiseBase<_Value, _Error>;
  using LaunchAsync = Base::LaunchAsync;
  using Cancel = Base::Cancel;
  using State = Base::State;

  Promise() : Base() {
    /// The Promise is a member of the coroutine state, so we can effectively track coroutine state
    /// allocations inside the promise
/// coro_handle::from_address is an ugly-ass pointer arithmetic hack to find out the address of a
/// struct if you know the address of a member of it that somehow got standartized
#ifdef ICEY_CORO_DEBUG_PRINT
    std::cout << fmt::format("Created coroutine state: 0x{:x} (Promise {})\n",
                             size_t(std::coroutine_handle<Self>::from_promise(*this).address()),
                             get_type(*this))
              << " from thread " << std::this_thread::get_id() << std::endl;

#endif
  }
  Task<_Value, _Error> get_return_object() noexcept;

  std::suspend_never final_suspend() const noexcept { return {}; }
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
  using Self = Promise<void, Nothing>;
  Promise() : Base() {
    /// The Promise is a member of the coroutine state, so we can effectively track coroutine state
    /// allocations inside the promise
/// coro_handle::from_address is an ugly-ass pointer arithmetic hack to find out the address of a
/// struct if you know the address of a member of it that somehow got standartized
#ifdef ICEY_CORO_DEBUG_PRINT
    std::cout << fmt::format("Created coroutine state: 0x{:x} (Promise {})\n",
                             size_t(std::coroutine_handle<Self>::from_promise(*this).address()),
                             get_type(*this))
              << std::endl;
#endif
  }

  std::suspend_never final_suspend() const noexcept { return {}; }

  Task<void, Nothing> get_return_object() noexcept;
  auto return_void() { this->resolve(Nothing{}); }
};

template <class _Value, class _Error = Nothing>
class [[nodiscard]] Task {
public:
  using Self = Task<_Value, _Error>;
  using Value = _Value;
  using Error = _Error;
  using PromiseT = Promise<_Value, _Error>;
  using promise_type = PromiseT;
  using LaunchAsync = PromiseT::LaunchAsync;
  std::unique_ptr<PromiseT> local_promise_;  /// for cB api

  /// Sets a function that launches the asynchronous operation: This handler is called in the with
  /// the address to this Promise so that it can store it and write to this promise later. This
  /// handler returns a cancellation function that gets called when this Promise is destructed. This
  /// constructor is useful for wrapping an existing callback-based API.
  explicit Task(LaunchAsync &&h) {
    local_promise_ = std::make_unique<PromiseT>();
    local_promise_->launch_async_ = h;
#ifdef ICEY_CORO_DEBUG_PRINT
    std::cout << get_type(*this) << " Task(launch_async)-" << std::endl;
#endif
  }

  explicit Task(std::coroutine_handle<PromiseT> handle) : coroutine_(handle) {
#ifdef ICEY_CORO_DEBUG_PRINT
  // fmt::print("{} Constructor from coroutine: 0x{:x}\n", get_type(*this),
  // std::size_t(handle.address()));
#endif
  }

  Task(const Task &) = delete;
  Task(Task &&other) = delete;
  Task &operator=(const Task &) = delete;
  Task &operator=(Task &&other) = delete;

  ~Task() {
#ifdef ICEY_CORO_DEBUG_PRINT
    std::cout << get_type(*this) << " ~Task() ";
    if (coroutine_) {
      std::cout << fmt::format("coroutine 0x{:x} is done: {}", std::size_t(coroutine_.address()),
                  coroutine_.done())
          << std::endl;
    } else {
      std::cout << " (Promise)" << std::endl;
    }
#endif
  }

  auto operator co_await() noexcept {
    struct Awaiter {
      PromiseT *promise_{nullptr};
      std::coroutine_handle<PromiseT> coroutine_{nullptr};
      Awaiter(PromiseT *promise) noexcept : promise_(promise) {}
      Awaiter(std::coroutine_handle<PromiseT> coroutine) noexcept : coroutine_(coroutine) {}

      bool await_ready() const noexcept {
        /// If we co_return handler, the coro is immediately fulfilled and done and we shall not
        /// suspend
#ifdef ICE Y_CORO_DEBUG_PRINT
        if (coroutine_) {
          std::cout << fmt::format("await_ready(), coroutine 0x{:x} is done: {}",
                                   std::size_t(coroutine_.address()), coroutine_.done())
                    << std::endl;
        }
#endif
        return false;
      }

      auto await_suspend(std::coroutine_handle<> awaiting_coroutine) noexcept {
        if (coroutine_) {
          auto &p = coroutine_.promise();
#ifdef ICEY_CORO_DEBUG_PRINT
          std::cout << fmt::format("{} await_suspend(), coroutine 0x{:x} is awaited by 0x{:x}\n",
                                   get_type(p), std::size_t(coroutine_.address()),
                                   std::size_t(awaiting_coroutine.address()))
                    << std::endl;
#endif
          p.launch_async(awaiting_coroutine);
        } else {
          promise_->launch_async(awaiting_coroutine);
#ifdef ICEY_CORO_DEBUG_PRINT
          std::cout << "await_suspend() called on promise task" << std::endl;
#endif
        }
        return true;
      }

      auto await_resume() {
        if (coroutine_) {
          auto &promise = this->coroutine_.promise();
          return promise.get();
        } else {
#ifdef ICEY_CORO_DEBUG_PRINT
          std::cout << get_type(*promise_) << " await_resume()" << std::endl;
#endif
          return promise_->get();
        }
      }
    };
    if (coroutine_) return Awaiter{coroutine_};
    return Awaiter{this->local_promise_.get()};
  }

  void resume () {
    coroutine_.resume();
  }
private:
  std::coroutine_handle<PromiseT> coroutine_{nullptr};
};

template <class Value, class Error>
inline Task<Value, Error> Promise<Value, Error>::get_return_object() noexcept {
#ifdef ICEY_CORO_DEBUG_PRINT
  // std::cout << get_type(*this) << " get_return_object() " << std::endl;
#endif
  return Task<Value, Error>{std::coroutine_handle<Promise<Value, Error>>::from_promise(*this)};
}

inline Task<void, Nothing> Promise<void, Nothing>::get_return_object() noexcept {
#ifdef ICEY_CORO_DEBUG_PRINT
  // std::cout << get_type(*this) << " get_return_object() " << std::endl;
#endif
  return Task<void, Nothing>{std::coroutine_handle<Promise<void, Nothing>>::from_promise(*this)};
}

}  // namespace icey