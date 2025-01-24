#include <icey/impl/observable.hpp>
#include <coroutine>

#include <iostream>
#include <thread>


#include <chrono>

using namespace std::chrono_literals;
 
namespace icey {

template <typename _Value, typename _ErrorValue = Nothing>
struct AwaitablePromise  {

  using Impl = impl::Observable<_Value, _ErrorValue>;
  using Value = typename Impl::Value;
  using MaybeValue = typename Impl::MaybeValue;
  using ErrorValue = typename Impl::ErrorValue;
  using Self = AwaitablePromise<_Value, _ErrorValue>;
  
  using promise_type = Self;
    Self get_return_object() { 
      //auto child = Self::create_from_impl(observable_->then(f));
        std::cout << "get_return_object called, this is " << std::hex << size_t(this) << std::endl;
        return *this; 
    }
    std::suspend_never initial_suspend() { 
        std::cout << "initial_suspend  " << std::endl;
        return {}; 
    }
    /// TODO enable if Value == Nothing
    //void return_void() {}
    /// TODO enable if Value != Nothing
    Value return_value() {
      return get_promise()->value();
    }
    void unhandled_exception() {}
    std::suspend_always final_suspend() noexcept { 
        std::cout << "final_suspend  " << std::endl;
        return {}; 
    }
    

    AwaitablePromise() {
      std::cout << "Ctor called , this is " << std::hex << size_t(this) << std::endl;
    }
  /// Pattern-maching factory function that creates a New Self with different value and error types
  /// based on the passed implementation pointer. 
  template <class NewVal, class NewErr>
  static AwaitablePromise<NewVal, NewErr> create_from_impl(
      std::shared_ptr<impl::Observable<NewVal, NewErr>> obs_impl) {
    //auto new_obs = impl::create_observable<AwaitablePromise<NewVal, NewErr>>();
    AwaitablePromise<NewVal, NewErr> new_obs;
    new_obs.observable_ = obs_impl;
    return new_obs;
  }
  std::shared_ptr<Impl> observable_{impl::create_observable<Impl>()};
  std::shared_ptr<Impl> get_promise() const {return this->observable_;}


  [[nodiscard]] auto operator co_await() const noexcept {
			struct awaiter {
				explicit awaiter(std::shared_ptr <Impl> state) : m_state(state) {}
				[[nodiscard]] bool await_ready() noexcept {
					return m_state->has_value();
				}
				[[nodiscard]] bool await_suspend(std::coroutine_handle<AwaitablePromise> handle) noexcept {
					
					
					if (m_state->has_value()) {
						m_state->register_handler([]() { 
              std:: cout << "Sleeping for 2s ... " << std::endl;
                std::this_thread::sleep_for(2000ms);
                std:: cout << "Awake again ! " << std::endl;
              });
						return true;
					}
					return false;
				}
				[[nodiscard]] std::string await_resume() {
					std:: cout << "Have value: "  << m_state->has_value() << std::endl;
					if (m_state->has_value())
						return m_state->value();
          else if (m_state->has_error())
            return m_state->error();
          return "";
				}

			private:
				std::shared_ptr <Impl> m_state;
			};

 
			return awaiter{observable_};
		}

};

}

using ResolveValue = std::string;
using ErrorValue = std::string;
using APromise = icey::AwaitablePromise<ResolveValue, ErrorValue>;

APromise test_async_await() {
  std::cout << "test_async_await  " << std::endl;
  //co_return;
  co_await APromise {};
}

int main(int argc, char **argv) {
  
  std::cout << "Starting  " << std::endl;
  APromise demo_instance = test_async_await();
  

}

