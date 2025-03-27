# Timers 

Timers are Streams as well, they behave similarly to subscribers. 

To create a timer:
```cpp
icey::TimerStream my_timer = node->icey().create_timer(100ms);
```

Key differences to regular ROS are: 
     - `create_timer` returns a `Stream` (you can access the `rclcpp::TimerBase` using `stream.timer`)
    - Asynchronous functions, i.e. coroutines, can be used as callbacks
     - The lifetime of the timer is bound to the lifetime of the node: This means, you don't need to store the timer as a member in the node class (i.e. do bookkeeping)
     
We can easily register a callback and use the timer like you usually would: 

```cpp
auto my_timer = node->icey().create_timer(100ms);

my_timer.then([](size_t ticks) {
    /// Do work
});
```

The `ticks` value in the callback indicates the number of times this timer ticked (starting from zero at the first call).