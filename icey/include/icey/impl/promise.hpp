/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

#pragma once

#include <coroutine>
#include <exception>  /// for std::exception_ptr
#include <functional>
#include <icey/impl/result.hpp>

#ifdef ICEY_CORO_DEBUG_PRINT
#include <fmt/format.h>

#include <boost/type_index.hpp>
#include <iostream>
#include <thread>
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

inline bool icey_coro_debug_print = false;

template <class Value, class Error>
class Promise;

/// A special type that indicates that there is no value. (Using `void` for this would cause many
/// problems, so defining an extra struct is easier.)
struct Nothing {};

namespace impl {

template <typename, typename = std::void_t<>>
struct has_promise_type : std::false_type {};

template <typename T>
struct has_promise_type<T, std::void_t<typename T::promise_type>> : std::true_type {};

template <typename T>
inline constexpr bool has_promise_type_v = has_promise_type<T>::value;
struct PromiseTag {};

/// This state is a sum type that holds either a Value, Error, or none. 
template <class _Value, class _Error>
struct PromiseState : private std::variant<std::monostate, _Value, _Error> {
  using Value = _Value;
  using Error = _Error;
  using Self = PromiseState<_Value, _Error>;
  
  PromiseState() = default;
  PromiseState(const Result<Value, Error> &result) { // NOLINT
    if(result.has_value()) {
      set_value(result.value());
    } else {
      set_error(result.error());
    }
  }

  bool has_none() const { return this->index() == 0; }
  bool has_value() const { return this->index() == 1; }
  bool has_error() const { return this->index() == 2; }
  const Value &value() const { return std::get<1>(*this); }
  const Error &error() const { return std::get<2>(*this); }
  void set_none() { this->template emplace<0>(std::monostate{}); }
  void set_value(const Value &x) { this->template emplace<1>(x); }
  void set_error(const Error &x) { this->template emplace<2>(x); }

  auto get() const {
    if constexpr (std::is_same_v<Error, Nothing>) {
      return this->value();
    } else {
      return to_result();
    }
  }

  Result<Value, Error> to_result() const {
    if(has_value()) {
        return Ok(value());
    } else  {
        return Err(error());
    }
  }
};

/// A Promise is an asynchronous abstraction that yields a value or an error.
/// It can be used with async/await syntax coroutines in C++20.
/// It also allows for wrapping an existing callback-based API.
/// It does not use dynamic memory allocation to store the value.
template <class _Value = Nothing, class _Error = Nothing>
class PromiseBase : public PromiseTag {
public:
  using Value = _Value;
  using Error = _Error;
  using State = PromiseState<_Value, _Error>;
  using Self = PromiseBase<Value, Error>;

  using Cancel = std::function<void(Self &)>;
  using LaunchAsync = std::function<void(Self &)>;
  LaunchAsync launch_async_;
  PromiseBase(LaunchAsync l) : launch_async_(l) {}

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

  /// Returns always false, we always first have to go to the executor to yield something. Used only
  /// when wrapping a callback-based API.
  bool await_ready() const noexcept { return false; }
  /// Launches the asynchronous operation that was previously stored and sets the awaiting coroutine
  /// as continuation. Suspends the current coroutine, i.e. returns true. Used only when wrapping a
  /// callback-based API.
  auto await_suspend(std::coroutine_handle<> awaiting_coroutine) noexcept {
    if (!this->launch_async_) {
      /// TODO throw something, this promise cannot be fulfilled
    }
    this->continuation_ = awaiting_coroutine;
    this->launch_async_(*this);
#ifdef ICEY_CORO_DEBUG_PRINT
    std::cout << fmt::format(
                     "PromiseBase await_suspend(), awaiting coroutine is 0x{:x}, "
                     "promise: {}\n",
                     std::size_t(awaiting_coroutine.address()), get_type(*this))
              << std::endl;
#endif
    return true;
  }

  /// get()s this promise. Used only when wrapping a callback-based API.
  auto await_resume() { return get(); }

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
        // TODOthrow std::runtime_error("Promise has nothing, called resume too early.");
      }
      return get_state().get();
    }
  }

  /// We do not use structured concurrency approach, we want to start the coroutine directly.
  std::suspend_never initial_suspend() const noexcept { return {}; }
  /// We do not suspend at the final suspend point, but continue so that the coroutine state is
  /// destructed automaticall when the coroutine is finished.
  std::suspend_never final_suspend() const noexcept { return {}; }

  /// Set the cancellation function that is called in the destructor if this promise has_none().
  void set_cancel(Cancel cancel) { cancel_ = cancel; }
  void set_continuation(std::coroutine_handle<> continuation) { continuation_ = continuation; }

protected:
  /// State of the promise: May be nothing, value or error.
  State state_;

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
  using State = Base::State;
  using Base::Base;

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

  ::icey::Promise<_Value, _Error> get_return_object() noexcept;

  /// return_value (aka. operator co_return) *sets* the value if called with an argument,
  /// very confusing, I know
  /// (Reference: https://devblogs.microsoft.com/oldnewthing/20210330-00/?p=105019)
  void return_value(const _Value &x) { this->resolve(x); }

  /// Sets the state to the given one:
  void return_value(const State &x) { this->put_state(x); }
};

/// Specialization so that Promise<void> works. (Nothing interesting here, just C++ 20 coroutine
/// specification being weird. There was an attempt to make it less weird, unfortunately
/// unsuccessful: https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2019/p1713r0.pdf)
template <>
class Promise<void, Nothing> : public PromiseBase<Nothing, Nothing> {
public:
  using Base = PromiseBase<Nothing, Nothing>;
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

  ::icey::Promise<void, Nothing> get_return_object() noexcept;
  auto return_void() { this->resolve(Nothing{}); }
};
}  // namespace impl

/// This is the type that users writing coroutines use as the return type. It is what is returned
/// when calling promise_type::get_return_value(). It is necessary because of the C++20 coroutines
/// spec that apparently tries to optimize the copying of the promise inside the coroutine state to
/// the caller. To not confuse the users with C++ coroutine spec's intricacies, we just call this
/// "Promise". Note that this is not a "Task": this term is used seemingly exclusively for the
/// structured programming approach of lazily started coroutines. I.e. it is not a Task as
/// implemented in in Lewis Bakers's cppcoro library and described in
/// https://www.open-std.org/JTC1/SC22/WG21/docs/papers/2018/p1056r1.html.
/// We cannot use the structured programming approach because it requires a custom executor but we
/// want to use the existing ROS executor.
template <class _Value, class _Error = Nothing>
class Promise {
public:
  using Self = Promise<_Value, _Error>;
  using Value = _Value;
  using Error = _Error;

  using promise_type = impl::Promise<_Value, _Error>;

  explicit Promise(std::coroutine_handle<promise_type> coroutine) : coroutine_(coroutine) {
#ifdef ICEY_CORO_DEBUG_PRINT
    std::cout << fmt::format("{} Constructor from coroutine: 0x{:x}\n", get_type(*this),
                             std::size_t(coroutine.address()))
              << std::endl;
#endif
  }

  Promise(const Promise &) = delete;
  Promise(Promise &&other) = delete;
  Promise &operator=(const Promise &) = delete;
  Promise &operator=(Promise &&other) = delete;

  ~Promise() {
#ifdef ICEY_CORO_DEBUG_PRINT
    std::cout << fmt::format("{} Destructor from coroutine: 0x{:x}\n", get_type(*this),
                             std::size_t(coroutine_.address()))
              << std::endl;
#endif
  }

  bool await_ready() const noexcept { return false; }

  auto await_suspend(std::coroutine_handle<> awaiting_coroutine) noexcept {
    auto &p = coroutine_.promise();
#ifdef ICEY_CORO_DEBUG_PRINT
    std::cout << fmt::format("{} await_suspend(), coroutine 0x{:x} is awaited by 0x{:x}\n",
                             get_type(p), std::size_t(coroutine_.address()),
                             std::size_t(awaiting_coroutine.address()))
              << std::endl;
#endif
    p.set_continuation(awaiting_coroutine);
    return true;
  }

  auto await_resume() { return coroutine_.promise().get(); }

private:
  std::coroutine_handle<promise_type> coroutine_{nullptr};
};

namespace impl {
template <class Value, class Error>
inline ::icey::Promise<Value, Error> Promise<Value, Error>::get_return_object() noexcept {
  return ::icey::Promise<Value, Error>{
      std::coroutine_handle<Promise<Value, Error>>::from_promise(*this)};
}
inline ::icey::Promise<void, Nothing> Promise<void, Nothing>::get_return_object() noexcept {
  return ::icey::Promise<void, Nothing>{
      std::coroutine_handle<impl::Promise<void, Nothing>>::from_promise(*this)};
}
}  // namespace impl

}  // namespace icey