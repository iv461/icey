## Timers 

Timers are Streams as well, they behave similarly to subscribers. 
To create a timer:
```cpp
icey::TimerStream my_timer = node->icey().create_timer(100ms);
```

Key differences to regular ROS are: 
     - Timers are `Stream`s
     - Asynchronous functions, i.e. coroutines, can be used as callbacks
     - The lifetime of the subscription is bound to the lifetime of the node: This means, you don't need to store  the subscriber as a member in the node class (i.e. do bookkeeping)
     - Asynchronous functions, i.e. coroutines may be used as callbacks 
     - 
We can easily register a callback and use the timer like you usually would: 

```cpp
auto my_timer = node->icey().create_timer(100ms);

my_timer.then([](size_t ticks) {
    /// Do work
});
```