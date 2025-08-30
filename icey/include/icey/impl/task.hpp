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

  auto initial_suspend() noexcept { return std::suspend_always{}; }

  auto final_suspend() noexcept { return final_awaitable{}; }

  auto continuation(std::coroutine_handle<> continuation) noexcept -> void {
    m_continuation = continuation;
  }

protected:
  std::coroutine_handle<> m_continuation{nullptr};
};

template <typename return_type>
struct promise final : public promise_base {
private:
  struct unset_return_value {
    unset_return_value() {}
    unset_return_value(unset_return_value&&) = delete;
    unset_return_value(const unset_return_value&) = delete;
    auto operator=(unset_return_value&&) = delete;
    auto operator=(const unset_return_value&) = delete;
  };

public:
  using task_type = task<return_type>;
  using coroutine_handle = std::coroutine_handle<promise<return_type>>;
  static constexpr bool return_type_is_reference = std::is_reference_v<return_type>;
  using stored_type =
      std::conditional_t<return_type_is_reference, std::remove_reference_t<return_type>*,
                         std::remove_const_t<return_type>>;
  using variant_type = std::variant<unset_return_value, stored_type, std::exception_ptr>;

  promise() noexcept {}
  promise(const promise&) = delete;
  promise(promise&& other) = delete;
  promise& operator=(const promise&) = delete;
  promise& operator=(promise&& other) = delete;
  ~promise() = default;

  auto get_return_object() noexcept -> task_type;

  auto return_value(const stored_type& value) -> void requires(not return_type_is_reference) {
    m_storage.template emplace<stored_type>(value);
  }

  auto return_value(std::function<void(promise<return_type>&)>&& h) {
    h(*this);
#ifdef ICEY_CORO_DEBUG_PRINT
    std::cout << "return_value handle-constructed: " << get_type(*this) << std::endl;
#endif
  }

  auto resolve(const stored_type& value) {
    return_value(value);
#ifdef ICEY_CORO_DEBUG_PRINT
    std::cout << "resolved, continuing ... " << get_type(*this) << std::endl;
#endif
    coroutine_handle::from_promise(*this).resume();
  }

  auto unhandled_exception() noexcept -> void {
    new (&m_storage) variant_type(std::current_exception());
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
  variant_type m_storage{};
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

    auto await_ready() const noexcept -> bool { return !m_coroutine || m_coroutine.done(); }

    auto await_suspend(std::coroutine_handle<> awaiting_coroutine) noexcept
        -> std::coroutine_handle<> {
      auto& p = m_coroutine.promise();
#ifdef ICEY_CORO_DEBUG_PRINT
      std::cout << "task await_suspend called on promise: " << get_type(p) << std::endl;
#endif
      p.continuation(awaiting_coroutine);
      return m_coroutine;
    }

    std::coroutine_handle<promise_type> m_coroutine{nullptr};
  };

  task() noexcept : m_coroutine(nullptr) {}

  explicit task(coroutine_handle handle) : m_coroutine(handle) {}
  task(const task&) = delete;
  task(task&& other) noexcept : m_coroutine(std::exchange(other.m_coroutine, nullptr)) {}

  ~task() {
#ifdef ICEY_CORO_DEBUG_PRINT
    auto& p = m_coroutine.promise();
    std::cout << "~task destroying promise: " << get_type(p) << std::endl;
#endif
    if (m_coroutine != nullptr) {
      m_coroutine.destroy();
    }
  }

  auto operator=(const task&) -> task& = delete;

  auto operator=(task&& other) noexcept -> task& {
    if (std::addressof(other) != this) {
      if (m_coroutine != nullptr) {
        m_coroutine.destroy();
      }

      m_coroutine = std::exchange(other.m_coroutine, nullptr);
    }

    return *this;
  }

  /**
   * @return True if the task is in its final suspend or if the task has been destroyed.
   */
  auto is_ready() const noexcept -> bool { return m_coroutine == nullptr || m_coroutine.done(); }

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

  auto operator co_await() const& noexcept {
    struct awaitable : public awaitable_base {
      auto await_resume() -> decltype(auto) { return this->m_coroutine.promise().result(); }
    };

    return awaitable{m_coroutine};
  }

  auto operator co_await() const&& noexcept {
    struct awaitable : public awaitable_base {
      auto await_resume() -> decltype(auto) {
        return std::move(this->m_coroutine.promise()).result();
      }
    };

    return awaitable{m_coroutine};
  }

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