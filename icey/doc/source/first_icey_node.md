# Your first ICEY-Node 

In the following, we will assume you are already familiar writing ROS nodes in C++: Creating nodes and using subscribers, publishers and callbacks. 

## Signal generator example

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

In this simple example we already see some interesting features: You do not need to create a publisher beforehand, instead you declare that the result should be published on a topic. 
Also, we do not store a timer object anywhere, ICEY stores it internally so that it does not get out of scope. This holds true for classes as well: In ICEY, you do not have to store 
subscribers/timers/publisher etc. as members of the class, ICEY does this bookkeeping for you. 

## The ICEY-node 

The ICEY library has two node classes: `icey::Node` and `icey::LifecycleNode`. 
To create new nodes, you use the `icey::create_node(argc, argv, <node_name>)` function. This function simply creates a node with `std::make_shared`, but calls `rclcpp::init` beforehand if needed, so that you don't have to do it.

The `icey::Node` is a subclass of an `rclcpp::Node`, so that you can drop-in replace you existing Node and gradually adopt ICEY's features.
ICEY offers all it's features through the `icey::Context` interface, accessed through `node->icey()`.

Since the ICEY-nodes are just subclasses of regular nodes, you can also build them into shared libraries and load them at runtime, i.e. use them as *components* with the `RCLCPP_COMPONENTS_REGISTER_NODE`-macro

ICEY represents ROS primitives (sub/pub etc.) as a `Stream`, an asynchronous programming abstraction for a sequence of values. Such `Stream`s can be used with async/await syntax, i.e. `co_awaited` to obtain a new message from a subscriber for example. 

If you are familiar with JavaScript, this is essentially a Promise, only that the state transitions are not final.

In the following, we will look more closely into how Subscribers and Timers follow the `Stream` concept and how this changes the way of asynchronous programming. 

## References: 

- [1] https://discourse.ros.org/t/simplifying-how-to-declare-parameters-in-ros-2/33272
- [2] Promise proposal by William Woodall and async/await discussion: https://github.com/ros2/ros2_documentation/issues/901
- [3] Mozilla Promise implementation: https://firefox-source-docs.mozilla.org/xpcom/mozpromise.html
- [4] std::future lacks continuation, current (2024) state of the art: https://ikriv.com/blog/?p=4916
- [5] https://engineering.fb.com/2015/06/19/developer-tools/futures-for-c-11-at-facebook/
- [1] Tutorial on how to use callback groups in rclcpp https://discourse.ros.org/t/how-to-use-callback-groups-in-ros2/25255