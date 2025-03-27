# Getting started

ICEY is a a new API for the Robot Operating System (ROS) 2 that allows for modern asynchronous programming using Streams, Promises and C++20 coroutines with async/await syntax. This simplifies application code and makes the asynchronous data-flow clearly visible. This enables fast prototyping with less boilerplate code.

It is fully compatible to the ROS 2 API and allows for gradual adoption since the `icey::Node` extends a regular ROS-Node. It supports all major features of ROS: parameters, subscribers, publishers, timers, services, clients, TF. It supports not only regular nodes but also lifecycle nodes with a single API. 

ICEY operates smoothly together with the  `message_filters` package, and it uses it for synchronization. ICEY also allows for extension, demonstrated by the the support for `image_transport` camera subscriber/publishers that is already implemented.

It offers additional goodies such as:
- Automatic bookkeeping of publishers/subscribers/timers so that you do not have to do it 
- No callback groups needed for preventing deadlocks -- async/await allows for synchronously looking code while the service calls remain asynchronous
- Handle many parameters easily with a single parameter struct that is registered automatically using static reflection, so that you do not need to repeat yourself

ICEY supports ROS 2 Humble and ROS 2 Jazzy.

The [icey_examples](../../icey_examples) package contains many different example nodes, demonstrating the capabilities of ICEY.

# Install 

ICEY comes as a regular ROS 2 package, to install it just clone it in you colcon workspace and build it:

TODO update link 

```sh
git clone https://github.com/DriverlessMobility/icey.git
sudo apt install libboost-dev libfmt-dev
MAKEFLAGS="-j4" colcon build --packages-select icey icey_examples --cmake-args -DCMAKE_BUILD_TYPE=Release
```

# Your first ICEY-Node 

In the following, we will assume you are already familiar writing ROS nodes in C++ (See [Official Client library Tutorials](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries.html)).

The ICEY library has two node classes: `icey::Node` and an `icey::LifecycleNode`. 
To create new nodes, you use the `icey::create_node(argc, argv, <node_name>)` function. This function does simply create a node with `std::make_shared`, but calls `rclcpp::init` beforehand if needed, so that you don't have to do it.

```cpp
#include <icey/icey.hpp>

int main(int argc, char **argv) {
  auto node = icey::create_node(argc, argv, "sine_generator");

  node->icey().create_timer(100ms)
    .then([](size_t ticks) {
        /// This function gets called each time the timer ticks
        std_msgs::msg::Float32 float_val;
        double period_time_s = 0.1;
        double frequency = 3.;
        /// We can access the parameter value by implicit conversion (or explicitly using .value())
        double y = amplitude * std::sin((2 * M_PI) * frequency * (period_time_s * ticks));
        float_val.data = y;
        return float_val;
    })
    /// The returned value is published on the topic "sine_signal" after the timer ticked.
    .publish("sine_signal");
    icey::spin(node);
}
```

See also the [signal generator example](../../icey_examples/src/signal_generator.cpp).

In this simple example, we can already see some interesting features:
ICEY represents ROS primitives such as timers as a `Stream`, an abstraction over an asynchronous sequence of values. Streams allow for calling `.then` 
If you are familiar with JavaScript, this is essentially a promise, except that the state transitions are not final.
Such streams allow calls to `publish', i.e. they can be published directly. 
You do not need to create a publisher first, you just declare that the result should be published to a topic. 
Finally, we do not need to store the timer object anywhere, because the lifetime of entities in ICEY is bound to the lifetime of the node. This is generally true for other entities as well: In ICEY, you do not need to store subscribers/timers/services as members of the class, ICEY does this bookkeeping for you.

## References 

- [1] https://discourse.ros.org/t/simplifying-how-to-declare-parameters-in-ros-2/33272
- [2] Promise proposal by William Woodall and async/await discussion: https://github.com/ros2/ros2_documentation/issues/901
- [3] Mozilla Promise implementation: https://firefox-source-docs.mozilla.org/xpcom/mozpromise.html
- [4] std::future lacks continuation, current (2024) state of the art: https://ikriv.com/blog/?p=4916
- [5] https://engineering.fb.com/2015/06/19/developer-tools/futures-for-c-11-at-facebook/
- [1] Tutorial on how to use callback groups in rclcpp https://discourse.ros.org/t/how-to-use-callback-groups-in-ros2/25255