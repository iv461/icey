# Getting started

ICEY is a a new API for the Robot Operating System (ROS) 2 that incorporates modern asynchronous programming concepts such as Promises and Streams and allows to use C++20 coroutines with async/await syntax. This simplifies application code and makes the asynchronous data-flow clearly visible. This enables fast prototyping with less boilerplate code.

It is fully compatible to the ROS 2 API and allows for gradual adoption as the `icey::Node` extends a regular ROS-NOde. It supports all major features of ROS: parameters, subscribers, publishers, timers, services, clients, TF. It supports not only regular nodes but also lifecycle nodes with a single API. 

ICEY operates smoothly together with the  `message_filters` package, and it uses it for synchronization. ICEY also allows for extension, demonstrated by the already implemented support for `image_transport` camera subscriber/publishers.

It offers additional goodies such as:
- Automatic bookkeeping of publishers/subscribers/timers so that you do not have to do it 
- No callback groups needed for preventing deadlocks -- service calls are always asynchronous
- Handle many parameters easily with a single parameter struct that is registered automatically using static reflection, so that you do not need to repeat yourself

ICEY supports ROS 2 Humble and ROS 2 Jazzy.

The [icey_examples](../../icey_examples) package contains many different example nodes, demonstrating the capabilities of ICEY.

# Install ICEY 

ICEY comes as a regular ROS 2 package, to install it just clone it in you colcon workspace and build it:

TODO update link 

```sh
git clone git@github.com:iv461/icey.git
sudo apt install liboost-dev libfmt-dev
colcon build  --packages-up-to icey icey_examples -DCMAKE_BUILD_TYPE=Release
```

In the following, 


# Your first ICEY-Node 

In the following, we will assume you are already familiar writing ROS nodes in C++: How to write subscribers, publishers, and using callbacks. 

The ICEY library has two node classes `icey::Node` and an `icey::LifecycleNode`. 
To create new nodes, you use the `icey::create_node(argc, argv, <node_name>)` function. This function does simply create a node with `std::make_shared`, but calls `rclcpp::init` beforehand if needed, so that you don't have to do it

```cpp
#include <icey/icey.hpp>

int main(int argc, char **argv) {
  auto node = icey::create_node(argc, argv, "sine_generator");

  node->icey().create_timer(100ms)
    .then([](size_t ticks) {
        /// This function gets called each time the timer ticks
        return std::sin(0.1 * ticks * 2 * M_PI);
    })
    /// The returned value is published on the topic "sine_signal" after the timer ticked.
    .publish("sine_signal");
    icey::spin(node);
}
```

See also the [signal generator example](../../icey_examples/src/signal_generator.cpp).

ICEY represents every ROS primitive (sub/pub etc.) as a `Stream`, an abstraction of an asynchronous sequence of values. 
If you are familiar with JavaScript, this is essentially a Promise, only that the state transitions are not final.

In this simple example we already see some interesting features: You do not need to create a publisher beforehand, instead you declare that the result should be published on a topic. 
Also, we do not store a timer object anywhere, ICEY stores it internally so that it does not get out of scope. This holds true for classes as well: In ICEY, you do not have to store 
subscribers/timers/publisher etc. as members of the class, ICEY does this bookkeeping for you. 


## Timers 

Timers are also signals:

```cpp
auto my_timer = node->icey().create_timer(100ms);
```

You can think of them as sources similar to subscribers but with no information, publishing periodically. 
We can easily register a callback and use the timer like you usually would: 

```cpp
auto my_timer = node->icey().create_timer(100ms);

my_timer.then([](size_t ticks) {
    /// Do work
});
```

## References 

- [1] https://discourse.ros.org/t/simplifying-how-to-declare-parameters-in-ros-2/33272
- [2] Promise proposal by William Woodall and async/await discussion: https://github.com/ros2/ros2_documentation/issues/901
- [3] Mozilla Promise implementation: https://firefox-source-docs.mozilla.org/xpcom/mozpromise.html
- [4] std::future lacks continuation, current (2024) state of the art: https://ikriv.com/blog/?p=4916
- [5] https://engineering.fb.com/2015/06/19/developer-tools/futures-for-c-11-at-facebook/
- [1] Tutorial on how to use callback groups in rclcpp https://discourse.ros.org/t/how-to-use-callback-groups-in-ros2/25255