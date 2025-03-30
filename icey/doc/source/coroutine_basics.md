# Coroutine basics

Before looking into services and TF, we will need to look into C++ 20 coroutines.
Coroutines are functions that allow for asynchronous programming using async/await syntax, currently the most popular method of doing asynchronous programming. Async/await is used with other programming languages like JavaScript, Rust and C#.

Coroutines contain any of the keywords `co_await`, `co_return` (or `co_yield`). 
A coroutine is a function that __looks like it's returning__ when it needs to wait for an asynchronous operation with `co_await`, this is called *suspending*. Then, once the asynchronous operation is done, it *jumps* back into the function, this is called *resuming*. When a coroutine resumes, it continues to execute the code that follows `co_await` statement.

To see what this means, let's look at an example: 

```cpp
/// This example demonstrates the fundamentals of coroutines: 
#include <icey/icey.hpp>
using namespace std::chrono_literals;

icey::Promise<void> a_coroutine(icey::Context &ctx) {
    std::cout << "2. Before create_timer " << std::endl;
    auto timer = ctx.create_timer(1s, true);
    std::cout << "3. Before awaiting timer  " << std::endl;
    co_await timer;
    std::cout << "4. After awaiting timer  " << std::endl;
    co_return;
}

int main(int argc, char **argv) {
    auto node = icey::create_node(argc, argv, "icey_coroutine_example");
    std::cout << "1. Before calling coroutine " << std::endl;
    a_coroutine(node->icey());
    std::cout << "5. After calling coroutine " << std::endl;
    rclcpp::spin(node);
    std::cout << "6. After sync_wait " << std::endl;
}
```

See [coroutine example](../../../icey_examples/src/coroutine_example.cpp)

The function `a_coroutine` is a coroutine in which we create a timer and await it (that's the asynchronous operation). 
In which order do you think the statements will be printed ?

```
1. Before calling coroutine
2. Before create_timer
3. Before awaiting timer
5. After calling coroutine
4. After awaiting timer
```


As you see, the function progresses as usual, there are no surprises up until the `3. Before awaiting timer` statement. After printing this statement, the coroutine hits `co_await` and suspends. This looks like the coroutine is returning -- `5. After calling coroutine` is printed. 

This behavior allows to reach the call to `rclcpp::spin` that spins the ROS executor (the event loop). After one second has passed, `4. After awaiting timer` is printed from the coroutine. This is the interesting part -- the control seemingly *jumps* back again to the coroutine *after* it initially returned (`5. After calling coroutine` was printed).

## Why are coroutines a good fit for ROS ?

Coroutines are perfectly suited to make the callback-based API of ROS more readable which is a typical event queue application. They allow for asynchronous programming in a single thread, i.e. without concurrency. 
Coroutines enable synchronously-looking service calls (among other asynchronous operations) inside callbacks -- something that was previously not possible with regular ROS.

Coroutines are essentially syntactic sugar to nested callbacks -- the compiler transforms everything that follows a `co_await` statement into a callback.

Here is a semantically equivalent code that is obtained after the compiler transforms the `co_await` statement:

```cpp
void a_coroutine(icey::Context &ctx) {
    std::cout << "2. Before create_timer " << std::endl;
    auto timer = ctx.create_timer(1s, true);
    std::cout << "3. Before awaiting timer  " << std::endl;
    timer.then([timer]() {
        std::cout << "4. After awaiting timer  " << std::endl;
    });
}
```




