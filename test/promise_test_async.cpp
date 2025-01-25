#include <icey/impl/observable.hpp>
#include <coroutine>

#include <iostream>
#include <thread>
#include <type_traits>

#include <chrono>

using namespace std::chrono_literals;
 
namespace icey {

struct PTag{};
template <typename _Value, typename _ErrorValue = Nothing>
struct AwaitablePromise : public PTag {

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

    /// We never already got something, we always first need to spin the ROS executor to get a message
    std::suspend_never initial_suspend() { return {}; }
    
    Value return_value() {
      return get_promise()->value();
    } 

    template<class ReturnType>
    ReturnType return_value(ReturnType x) { return x; }

    void unhandled_exception() {}

    std::suspend_always final_suspend() noexcept { 
        std::cout << "final_suspend  " << std::endl;
        return {}; 
    }
    
    /// Promisify a value if needed, meaning if it is not already a promise
    template<class ReturnType>
    auto await_transform(ReturnType x) {
      if constexpr(std::is_base_of_v<PTag, ReturnType>)
        return x;
      else
       return AwaitablePromise<ReturnType, Nothing>{};
    }

    AwaitablePromise() {
      std::cout << "[AwaitablePromise @ 0x" << std::hex << size_t(this) << std::dec << 
        " Constructor called" << std::endl;
    }


    ~AwaitablePromise() {
      std::cout << "[AwaitablePromise @ 0x" << std::hex << size_t(this) << std::dec << 
        " Destructor called" << std::endl;
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

  size_t counter_{0};

    bool await_ready() noexcept {
					return this->observable_->has_value();
				}

    bool await_suspend(auto handle) {
    std:: cout << this->counter_ << "Sleeping for 2s ... " << std::endl;
      std::this_thread::sleep_for(200ms);
      std:: cout << "Awake again ! " << std::endl;
      
      if constexpr(std::is_same_v< Value, int>)
        this->observable_->resolve(this->counter_++);
      else 
        this->observable_->resolve("success! " +std::to_string(this->counter_++));
      
      return false; /// Resume the current coroutine, see https://en.cppreference.com/w/cpp/language/coroutines
    }

    /// This is called after await_suspend returned. Meaning, after we got the value.
    auto await_resume() {
      auto promise = this->observable_;
      if constexpr(std::is_same_v< _ErrorValue, Nothing >) {
        auto result = promise->value();
          /// Reset the state since we consumed this value
          promise->set_none();
          return result;
        } else {
          auto result = promise->get_state();
          /// Reset the state since we consumed this value
          promise->set_none();
          return result;
        }
    }
};
}

using ResolveValue = std::string;
using ErrorValue = std::string;
using APromise = icey::AwaitablePromise<ResolveValue, ErrorValue>;

icey::AwaitablePromise<int, icey::Nothing> test_async_await() {
  std::cout << "test_async_await  " << std::endl;
  APromise my_sub;
  
  icey::Result<std::string, std::string> result = co_await my_sub;
  std::cout << "result: " << result.value() << std::endl;

  icey::AwaitablePromise<int, icey::Nothing> sub2;
  int result2 = co_await sub2;
  co_return result2;
  
  //co_return 3;
}

int main(int argc, char **argv) {
  
  std::cout << "Starting  " << std::endl;
  auto promise_ret = test_async_await();

  std::cout << "END: Have value: " << promise_ret.get_promise()->has_value() << std::endl;

  if(promise_ret.get_promise()->has_value()) {
    std::cout << "END value: " << promise_ret.get_promise()->value() << std::endl;
    
  }  

}

