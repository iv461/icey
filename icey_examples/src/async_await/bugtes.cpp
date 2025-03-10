/// This example shows how to use a publisher with async/await syntax.
/// The synchronous call to .publish() simply calls publish on the ROS publisher.
#include <icey/icey.hpp>
#include "std_msgs/msg/string.hpp"

#include <thread> 

using namespace std::chrono_literals;

//
std::future<void> g_fut; 

/// We can use coroutines:
icey::Future<std_msgs::msg::String> async_create_message() {
    std::cout << "1. In async_create_message " << std::endl;
    size_t ticks = co_await icey::Future<std::size_t>{[](auto &fut) {
        g_fut = std::async(std::launch::async, [&]{ 
            std::this_thread::sleep_for(1s);
            fut.put_value(3); 
        });
        return icey::Future<std::size_t>::Cancel{};
    }};
    std::cout << "2. After await timer " << std::endl;
    std_msgs::msg::String message;
    message.data = "hello " + std::to_string(ticks);
    co_return message;
}

icey::Future<int> talk() {        
    std::cout << "3. B4 await " << std::endl;
    std_msgs::msg::String message = co_await async_create_message();
    std::cout << "4. After await async_create_message " << message.data << std::endl;
    co_return 0;
}


int main(int argc, char **argv) {
  icey::icey_coro_debug_print = true;

  talk();
  std::cout << "5. After talks " << std::endl;
  g_fut.wait();
}