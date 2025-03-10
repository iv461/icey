/// This example shows how to use a publisher with async/await syntax.
/// The synchronous call to .publish() simply calls publish on the ROS publisher.
#include <iostream>
#include <thread> 
#include <future>
#include <variant> 
#include <functional>
#include <boost/type_index.hpp>
#include <coroutine>
#include <string> 
#include <sstream> 


using namespace std::chrono_literals;

namespace icey {
bool icey_coro_debug_print;

template <class _Value, class _ErrorValue>
struct Result : private std::variant<std::monostate, _Value, _ErrorValue> {
  using Value = _Value;
  using ErrorValue = _ErrorValue;
  using Self = Result<_Value, _ErrorValue>;
  static Self None() { return Result<_Value, _ErrorValue>{}; }
  static Self Ok(const _Value &x) {
    Self ret;
    ret.template emplace<1>(x);
    return ret;
  }
  static Self Err(const _ErrorValue &x) {
    Self ret;
    ret.template emplace<2>(x);
    return ret;
  }
  bool has_none() const { return this->index() == 0; }
  bool has_value() const { return this->index() == 1; }
  bool has_error() const { return this->index() == 2; }
  const Value &value() const { return std::get<1>(*this); }
  const ErrorValue &error() const { return std::get<2>(*this); }
  void set_none() { this->template emplace<0>(std::monostate{}); }
  void set_ok(const Value &x) { this->template emplace<1>(x); }
  void set_err(const ErrorValue &x) { this->template emplace<2>(x); }
};

template <class T>
struct crtp {
  T &underlying() { return static_cast<T &>(*this); }
  T const &underlying() const { return static_cast<T const &>(*this); }
};

template <class Derived>
struct PromiseInterfaceForCoroutines : public crtp<Derived> {
  using promise_type = Derived;

  PromiseInterfaceForCoroutines() {
    if (icey_coro_debug_print) std::cout << get_type_info() << " Constructor called" << std::endl;
  }

  ~PromiseInterfaceForCoroutines() {
    if (icey_coro_debug_print) std::cout << get_type_info() << " Destructor called" << std::endl;
    if (this->underlying().coro_) 
      this->underlying().coro_.destroy();
    this->underlying().coro_ = nullptr;
  }  
  std::string get_type_info() const {
    std::stringstream ss;
    auto this_class = boost::typeindex::type_id_runtime(*this).pretty_name();
    ss << "[" << this_class << " @ 0x" << std::hex << size_t(this);
    return ss.str();
  }

  std::coroutine_handle<> continuation_;

  /// We are still a promise
  Derived get_return_object() {
    // std::cout << get_type_info() <<   " get_return_object called" << std::endl;
    /// This trick is used everywhere, in Lewis Bakers tutorial, as well as in other libraries: 
    // [cppcoro] https://github.com/lewissbaker/cppcoro/blob/master/include/cppcoro/task.hpp#L456
    // [asyncpp] https://github.com/asyncpp/asyncpp/blob/master/include/asyncpp/task.h#L23
    // [libcoro] https://github.com/jbaldwin/libcoro/blob/main/include/coro/task.hpp
    // Essentially, we create here a "box"-object (of promise_type) that holds the our actual promise object, that is also a promise_type
    // Why we need to do this ? Apparently the compiler always needs to box the promises, assuming that promise_type and the implementation might be different types
    return Derived{std::coroutine_handle<Derived>::from_promise(this->underlying())};
  }

  std::suspend_never initial_suspend() {
    // std::cout << get_type_info() <<   " initial_suspend called" << std::endl;
    return {};
  }

  std::suspend_never final_suspend() const noexcept { 
    return {};
    /*
    struct final_awaitable {
      bool await_ready() const noexcept { return false; }
      
      std::coroutine_handle<> 
      await_suspend(std::coroutine_handle<Derived> coro) noexcept {
        std::cout << "Final suspend await_suspend"<< std::endl;
        return coro.promise().continuation_;
      }
      void await_resume() noexcept {}
    };
    return final_awaitable{};
        */
  }

  /// return_value returns the value of the Steam.
  auto return_value() { return this->underlying().value(); }

  void unhandled_exception() {
    /// Wanna hear a joke ? 
    /// 1. There is a function defined std::exception_ptr current_exception() noexcept;
    /// 2. It returns: using exception_ptr = /*unspecified*/ (since C++11) 
    /// ...
    //this->underlying().set_error(std::current_exception()->what());
  }

  /// If the return_value function is called with a value, it *sets* the value, makes sense
  /// right ? No ? Oh .. (Reference:
  /// https://devblogs.microsoft.com/oldnewthing/20210330-00/?p=105019)
  template <class T>
  void return_value(const T &x) {
    if (icey_coro_debug_print)
      std::cout << this->get_type_info() << " return value for "
                << boost::typeindex::type_id_runtime(x).pretty_name() << " called " << std::endl;
    this->underlying().set_value(x);
  }

};
struct Nothing{};
template<class _Value, class _Error = Nothing>
class Future : public PromiseInterfaceForCoroutines<Future<_Value, _Error>> {
public:
    using Value = _Value;
    using Error = _Error;

    using State = Result<Value, Error>;
    
    State state_;

    const Value &value() const { return state_.value(); }
    /// Sets the state to hold a value, but does not notify about this state change.
    void put_value(const Value &x) { 
        state_.set_ok(x); 
        if(handler)
            handler();
    }
    void set_value(const Value & x) {
        state_.set_ok(x); 
    }

    using Self = Future<Value, Error>;
    using Cancel = std::function<void(Self &)>;

    Future() {
      std::cout << "Future was default-constructed: " << 
      get_type(*this) << std::endl;
    }

    explicit Future(std::coroutine_handle<Self> coro) : coro_(coro) {}
    std::coroutine_handle<Self> coro_;

    Future(const Self &) = delete;
    Future(Self &&) = delete;
    Future &operator=(const Future &) = delete;
    Future &operator=(Future &&) = delete;
  
    explicit Future(std::function<Cancel(Self &)> &&h) { cancel_ = h(*this); }

    Value take() {
        return state_.value();
    }
    bool has_none() {return state_.has_none(); }

      
    template<class T>
    static std::string get_type(T &t) {
      std::stringstream ss;
      auto this_class = boost::typeindex::type_id_runtime(t).pretty_name();
      ss << "[" << this_class << " @ 0x" << std::hex << size_t(&t);
      return ss.str();
    }

    /// Await the future 
    auto operator co_await() {
      struct Awaiter {
        std::coroutine_handle<Self> coro_;
        Awaiter(std::coroutine_handle<Self> coro) : coro_(coro) {}
        bool await_ready() const noexcept { 
          std::cout << "Await ready on Future " << get_type(coro_.promise()) << " called" << std::endl;
          if(!coro_) {
            std::cout << "No coro, returning true " << std::endl;
            return true;
          } else {
            std::cout << "Coro handle is valid " << std::endl;
          }
          return !coro_.promise().has_none(); 
        }
        void await_suspend(std::coroutine_handle<> h) noexcept { 
          /// Resume the coroutine when this promise is done
          std::cout << "Await suspend was called, held Future: " << get_type(coro_.promise()) << std::endl;
          //std::cout << "And the future in the coro handle: " << get_type(h.promise()) << std::endl;
          if(!coro_) {
            std::cout << "No coro, returning false " << std::endl;
            //return coro_;
            return;
          } else {
            std::cout << "Coro handle is valid " << std::endl;
          }
          coro_.promise().handler = [h]() { if(h) h.resume(); };
          coro_.promise().continuation_ = h;
          //return coro_;
        }
        auto await_resume() const noexcept { 
          std::cout << "Await resume was called, held Future: " << get_type(coro_.promise()) << std::endl;
          if(!coro_) {
            std::cout << "No coro, guess I'll die now " << std::endl;
          } else {
            std::cout << "Coro handle is valid " << std::endl;
          }
          return coro_.promise().take(); 
        }
      };
      return Awaiter{this->coro_};
    }

    ~Future() {
      std::cout << "Future was destructed: " << get_type(*this) << std::endl;
      if(cancel_)
          cancel_(*this);
    }

    std::function<void()> handler;
    Cancel cancel_;
    
    bool is_done{false};
};
}

std::future<void> g_fut; 

/// We can use coroutines:
icey::Future<std::string> async_create_message() {
    std::cout << "1. In async_create_message " << std::endl;
    size_t ticks = co_await icey::Future<std::size_t>{[](auto &fut) {
        g_fut = std::async(std::launch::async, [&]{ 
            std::this_thread::sleep_for(1s);
            fut.put_value(3); 
        });
        return icey::Future<std::size_t>::Cancel{};
    }};
    std::cout << "2. After await timer " << std::endl;
    std::string message = "hello " + std::to_string(ticks);
    co_return message;
}

icey::Future<int> talk() {        
    std::cout << "3. B4 await " << std::endl;
    std::string message = co_await async_create_message();
    std::cout << "4. After await async_create_message " << message << std::endl;
    co_return 0;
}


int main(int argc, char **argv) {
  icey::icey_coro_debug_print = true;

  talk();
  std::cout << "5. After talks " << std::endl;
  g_fut.wait();
}