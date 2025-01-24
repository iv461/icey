#include <coroutine>
#include <iostream>
#include <memory>
class UserFacing {
  public:
  using Self = UserFacing;
  using promise_type = Self;

    Self get_return_object() { 
        std::cout << "get_return_object  " << std::endl;
        return *this; 
    }
    std::suspend_never initial_suspend() { 
        std::cout << "initial_suspend  " << std::endl;
        return {}; 
    }
    void return_void() {}
    void unhandled_exception() {}
    std::suspend_always final_suspend() noexcept { 
        std::cout << "final_suspend  " << std::endl;
        return {}; 
    }
    


    struct Impl {

    };
    std::shared_ptr < Impl> impl_;
};

UserFacing demo_coroutine() {
    std::cout << "hello, world" << std::endl;
    co_return;
}

int main() {
    std::cout << "Starting  " << std::endl;
    UserFacing demo_instance = demo_coroutine();
}