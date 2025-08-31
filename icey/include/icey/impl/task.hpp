#pragma once

#include <coroutine>
#include <exception>
#include <icey/impl/promise.hpp>
#include <stdexcept>
#include <utility>
#include <variant>

namespace coro {
template <typename return_type = void>
class task;

namespace detail {
struct promise_base {
  friend struct final_awaitable;
  struct final_awaitable {
    auto await_ready() const noexcept -> bool { return false; }

    template <typename promise_type>
    auto await_suspend(std::coroutine_handle<promise_type> coroutine) noexcept
        -> std::coroutine_handle<> {
      auto& promise = coroutine.promise();
#ifdef ICEY_CORO_DEBUG_PRINT
      std::cout << "promise_base await_suspend was called on promise: " << get_type(promise)
                << std::endl;
#endif
      // If there is a continuation call it, otherwise this is the end of the line.
      if (promise.m_continuation != nullptr) {
        return promise.m_continuation;
      } else {
        return std::noop_coroutine();
      }
    }

    auto await_resume() noexcept -> void {
      // no-op
    }
  };

  promise_base() noexcept {
#ifdef ICEY_CORO_DEBUG_PRINT
    std::cout << "promise_base was default-constructed: " << get_type(*this) << std::endl;
#endif
  }
  ~promise_base() {
#ifdef ICEY_CORO_DEBUG_PRINT
    std::cout << "promise_base was destructed: " << get_type(*this) << std::endl;
#endif
  }

  std::suspend_never initial_suspend() noexcept { return {}; }

  std::suspend_always final_suspend() noexcept { return {}; }

  std::coroutine_handle<> m_continuation{nullptr};
};

template <typename return_type>
struct promise final : public promise_base {
private:

public:
  using task_type = task<return_type>;
  using coroutine_handle = std::coroutine_handle<promise<return_type>>;

  promise() noexcept {}
  promise(const promise&) = delete;
  promise(promise&& other) = delete;
  promise& operator=(const promise&) = delete;
  promise& operator=(promise&& other) = delete;
  ~promise() = default;

  auto get_return_object() noexcept -> task_type;

  //auto return_value() { return this->value(); }
  /// return_value (aka. operator co_return) *sets* the value if called with an argument,
  /// very confusing, I know
  /// (Reference: https://devblogs.microsoft.com/oldnewthing/20210330-00/?p=105019)
  void return_value(const _Value &x) { this->resolve(x); }

  /// Sets the state to the given one:
  void return_value(const State &x) { this->put_state(x); }

  auto return_value(std::function<void(promise<return_type>&)>&& h) {
    h(*this);
#ifdef ICEY_CORO_DEBUG_PRINT
    std::cout << "return_value handle-constructed: " << get_type(*this) << std::endl;
#endif
  }

  auto resolve(const stored_type& value) {
    return_value(value);
#ifdef ICEY_CORO_DEBUG_PRINT
    std::cout << "resolved promise " << get_type(*this) << std::endl;
#endif
    if (m_continuation)
      m_continuation.resume();
    else
      std::cout << "This promise has no continuation !" << std::endl;

    // coroutine_handle::from_promise(*this).resume();
  }

  auto result() & -> decltype(auto) {
    if (std::holds_alternative<stored_type>(m_storage)) {
      if constexpr (return_type_is_reference) {
        return static_cast<return_type>(*std::get<stored_type>(m_storage));
      } else {
        return static_cast<const return_type&>(std::get<stored_type>(m_storage));
      }
    } else if (std::holds_alternative<std::exception_ptr>(m_storage)) {
      std::rethrow_exception(std::get<std::exception_ptr>(m_storage));
    } else {
      throw std::runtime_error{"The return value was never set, did you execute the coroutine?"};
    }
  }

  auto result() const& -> decltype(auto) {
    if (std::holds_alternative<stored_type>(m_storage)) {
      if constexpr (return_type_is_reference) {
        return static_cast<std::add_const_t<return_type>>(*std::get<stored_type>(m_storage));
      } else {
        return static_cast<const return_type&>(std::get<stored_type>(m_storage));
      }
    } else if (std::holds_alternative<std::exception_ptr>(m_storage)) {
      std::rethrow_exception(std::get<std::exception_ptr>(m_storage));
    } else {
      throw std::runtime_error{"The return value was never set, did you execute the coroutine?"};
    }
  }

  auto result() && -> decltype(auto) {
    if (std::holds_alternative<stored_type>(m_storage)) {
      if constexpr (return_type_is_reference) {
        return static_cast<return_type>(*std::get<stored_type>(m_storage));
      } else if constexpr (std::is_move_constructible_v<return_type>) {
        return static_cast<return_type&&>(std::get<stored_type>(m_storage));
      } else {
        return static_cast<const return_type&&>(std::get<stored_type>(m_storage));
      }
    } else if (std::holds_alternative<std::exception_ptr>(m_storage)) {
      std::rethrow_exception(std::get<std::exception_ptr>(m_storage));
    } else {
      throw std::runtime_error{"The return value was never set, did you execute the coroutine?"};
    }
  }

private:
 
};

template <>
struct promise<void> : public promise_base {
  using task_type = task<void>;
  using coroutine_handle = std::coroutine_handle<promise<void>>;

  promise() noexcept = default;
  promise(const promise&) = delete;
  promise(promise&& other) = delete;
  promise& operator=(const promise&) = delete;
  promise& operator=(promise&& other) = delete;
  ~promise() = default;

  auto get_return_object() noexcept -> task_type;

  auto return_void() noexcept -> void {}

  auto unhandled_exception() noexcept -> void { m_exception_ptr = std::current_exception(); }

  auto result() -> void {
    if (m_exception_ptr) {
      std::rethrow_exception(m_exception_ptr);
    }
  }

private:
  std::exception_ptr m_exception_ptr{nullptr};
};

}  // namespace detail

template <typename return_type>
class [[nodiscard]] task {
public:
  using task_type = task<return_type>;
  using promise_type = detail::promise<return_type>;
  using coroutine_handle = std::coroutine_handle<promise_type>;

  struct awaitable_base {
    awaitable_base(coroutine_handle coroutine) noexcept : m_coroutine(coroutine) {}

    auto await_ready() const noexcept -> bool {
      return false;
      return !m_coroutine || m_coroutine.done();
    }

    auto await_suspend(std::coroutine_handle<> awaiting_coroutine) noexcept {
      auto& p = m_coroutine.promise();
#ifdef ICEY_CORO_DEBUG_PRINT
      std::cout << "task await_suspend called on promise, setting continuation: " << get_type(p)
                << std::endl;
#endif
      p.m_continuation = awaiting_coroutine;
      std::cout << "m_coroutine is: " << std::size_t(m_coroutine.address()) << std::endl;
      // return m_coroutine;
      return false;
    }
    auto await_resume() { return this->m_coroutine.promise().result(); }

    std::coroutine_handle<promise_type> m_coroutine{nullptr};
  };

  task() noexcept : m_coroutine(nullptr) {}

  explicit task(coroutine_handle handle) : m_coroutine(handle) {}
  task(const task&) = delete;
  task(task&& other) = delete;

  /*~task() {

#ifdef ICEY_CORO_DEBUG_PRINT
    auto& p = m_coroutine.promise();
    std::cout << "~task destroying promise: " << get_type(p) << std::endl;
#endif
    if (m_coroutine != nullptr) {
      m_coroutine.destroy();
    }
  }*/

  task& operator=(const task&) = delete;
  task& operator=(task&& other)= delete;


  auto resume() -> bool {
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
  }

  auto operator co_await() const& noexcept { return awaitable_base{m_coroutine}; }

  auto promise() & -> promise_type& { return m_coroutine.promise(); }
  auto promise() const& -> const promise_type& { return m_coroutine.promise(); }
  auto promise() && -> promise_type&& { return std::move(m_coroutine.promise()); }

  auto handle() -> coroutine_handle { return m_coroutine; }

private:
  coroutine_handle m_coroutine{nullptr};
};

namespace detail {
template <typename return_type>
inline auto promise<return_type>::get_return_object() noexcept -> task<return_type> {
  return task<return_type>{coroutine_handle::from_promise(*this)};
}

inline auto promise<void>::get_return_object() noexcept -> task<> {
  return task<>{coroutine_handle::from_promise(*this)};
}

}  // namespace detail

}  // namespace coro

namespace icey {
template <typename return_type>
using task = coro::task<return_type>;
}