# Coroutine basics

Before looking into services and TF are handled in ICEY, we will look into C++ 20 coroutines.
Coroutines are functions that allow for asynchronous programming using async/await syntax, currently the most popular method of doing asynchronous programming. Async/await is used with other programming languages like JavaScript, Rust and C#.

Coroutines contain any of the keywords `co_await`, `co_return` (or `co_yield`). 
A coroutine is a function that can be paused and resumed later. When it reaches a `co_await` statement, it has to wait for an asynchronous operation and *suspends*. Then, once the asynchronous operation is done, it resumes after the `co_await` statement.

To see what this means, let's look at an example: 

```cpp
/// This example demonstrates the fundamentals of coroutines: 
#include <icey/icey.hpp>
using namespace std::chrono_literals;

icey::Promise<void> a_coroutine(icey::Context &ctx) {
    std::cout << "2. Before create_timer" << std::endl;
    auto timer = ctx.create_timer(1s, true);
    std::cout << "3. Before awaiting timer" << std::endl;
    co_await timer;
    std::cout << "4. After awaiting timer" << std::endl;
    co_return;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, arv);
    auto node = std::make_shared<icey::Node>("icey_coroutine_example");
    std::cout << "1. Before calling the coroutine" << std::endl;
    a_coroutine(node->icey());
    std::cout << "5. After calling the coroutine" << std::endl;
    rclcpp::executors::MultiThreadedExecutor exec{rclcpp::ExecutorOptions(), 8};
  exec.add_node(node->get_node_base_interface());
  exec.spin();
    std::cout << "6. After spin" << std::endl;
}
```

See [coroutine example](../../../icey_examples/src/coroutine_example.cpp)

The function `a_coroutine` creates a timer and awaits it, this is the asynchronous operation. 
The order in which the statements are printed is:

```
1. Before calling coroutine
2. Before create_timer
3. Before awaiting timer
5. After calling coroutine
4. After awaiting timer
```


As you see, the function progresses as usual, there are no surprises up until the `3. Before awaiting timer` statement. After printing this statement, the coroutine hits `co_await` and suspends, i.e. returns. After the coroutine returns, `5. After calling the coroutine` is printed. 

This behavior allows to reach the call to `rclcpp::spin` that spins the ROS executor (the event loop). After one second has passed, `4. After awaiting timer` is printed from the coroutine. This is the interesting part -- the control resumes back again to the coroutine *after* it initially returned (`5. After calling the coroutine` was printed).

## Why are coroutines a good fit for ROS ?

Coroutines are perfectly suited for simplifying the callback-based ROS-API.
Coroutines enable synchronously-looking service calls (among other asynchronous operations) inside callbacks -- something that was previously not possible with regular ROS.

Coroutines are essentially syntactic sugar to nested callbacks -- everything that follows a `co_await` statement becomes a new callback.





