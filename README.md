# ICEY 

A new, simple asynchronous/data-driven library for the Robot Operating System (ROS) for fast prototyping and eliminating boilerplate code.
ICEY enables using modern asynchronous programming techniques such as promises and C++-20's coroutines (async/await)

# Features 

The real power in ICEY is that you can declare computations, that will  be published automatically when the input changes: 

[Signal generator example](examples/signal_generator.cpp)
```cpp
#include <icey/icey.hpp>
int main(int argc, char **argv) {
    
    icey::create_timer(100ms)
      .then([&](size_t ticks) {
        /// We can access parameters in callbacks using .value() because parameters are always initialized first.
        double y = std::sin(0.1 * ticks * 2 * M_PI);
        return y;
    }).publish("sine_generator");

    /// Create and spin the node:
    icey::spawn(argc, argv, "signal_generator_example"); 
}
```

Using Streams (promises), you can build your own data-driven pipeline of computations, for example sequencing service calls: 

```cpp
icey::create_timer(1s)
    /// Build a request when the timer ticks
    .then([](size_t) {
        auto request = std::make_shared<ExampleService::Request>();
        request->data = true;
        RCLCPP_INFO_STREAM(icey::node->get_logger(),
                            "Timer ticked, sending request: " << request->data);
        return request;
    })
    /// Now call the service with the request build
    .call_service<ExampleService>("set_bool_service1", 1s)
    .then([](ExampleService::Response::SharedPtr response) {
        RCLCPP_INFO_STREAM(icey::node->get_logger(), "Got response1: " << response->success);
        auto request = std::make_shared<ExampleService::Request>();
        request->data = false;
        return request;
    })
    .call_service<ExampleService>("set_bool_service2", 1s)
    .then([](ExampleService::Response::SharedPtr response) {
        RCLCPP_INFO_STREAM(icey::node->get_logger(), "Got response2: " << response->success);
        ...
    })
    /// Here we catch timeout errors as well as unavailability of the service:
    .except([](const std::string& error_code) {
        RCLCPP_INFO_STREAM(icey::node->get_logger(), "Service got error: " << error_code);
    });
```     
This programming model is fully asynchronous and therefore there is danger of deadlocks when chaining multiple callbacks. 

ICEY is a thin wrapper around the public ROS 2 API, it does not reinvent anything or use private implementation details.

## Parameter declaration: 
ICEY also simplifies the declaration of many parameters: (very similar to the `dynamic_reconfigure`-package from ROS 1):

```cpp
/// All parameters of the node in a struct:
struct NodeParameters {
  /// We set a default value, allowed interval and a description
  icey::DynParameter<double> frequency{10., icey::Interval(0., 25.), std::string("The frequency of the sine")};
  icey::DynParameter<double> amplitude{3};
  icey::DynParameter<std::string> map_path{""};
  ...
};
 /// The object holding all the parameters:
 NodeParameters params;
  /// Declare parameter struct and receive updates each time a parameter changes:
  icey::declare_parameter_struct(params, [](const std::string &changed_parameter) {
        RCLCPP_INFO_STREAM(icey::node->get_logger(),
                           "Parameter " << changed_parameter << " changed");
      });
```

# Key features: 

- ICEY introduces modern asynchronous programming to ROS using Streams (Promises) and coroutines (async/await)
- ICEY minimizes boilerplate code needed for using parameters, creating and spawning nodes and synchronization 
- Automatic synchronization, unifying usage of TF as a form of synchronization
- Fully featured: Parameters, Pub/Sub, TF, Services, Lifecycle Nodes, `message_filters`, `image_transport` 
- Extensible: [We demonstrate](icey/doc/extending_icey.md) the extension of ICEY for custom `image_transport`-publishers/subscribers

- Easy asynchronous programming using promises, you do not [have to deal with callback groups in order to prevent deadlocks](https://docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html) or spawning extra threads

- Efficiency: No additional dynamic memory allocations compared to plain ROS happen after the node is initialized, also not for error handling thanks to using Result-types instead of C++-exceptions

# Performance: 

TODO summarize 

The promises implemented are generally very fast, they have a small (non-zero), but in practice neglible overhead compared to plain callbacks. 
To demonstrate this, we translated a typical node from the Autoware project with multiple subscribers/publishers and measured the performance with perf. 
The evaluation showed an overall latency increase of only X.X % and no significant increase of the latency variance (jitter). 
See the [Evaluation]-section for more details. 


# (small) limitations

We generally aim with ICEY to support everything that ROS also supports. 
Still, there are some small limitations: 

- Only the SingleThreadedExecutor is supported currently
- Memory strategy is not implemented, but could be easily
- Sub-nodes

# More features 

- C++20 coroutines, aka. Async/await: 
- 

# Dependencies: 

- ROS 2 (tested on Humble)
- Boost Hana
- C++20 ir required for the parameters struct feature and coroutine-support

# Build and test 

Simply clone into your ROS 2 workspace and build it. 

To build the examples, do:
```sh
colcon build  --packages-up-to icey_examples
```
To run the unit-tests:

```sh
colcon test --packages-select icey 
build/icey./promise_test
```

# Related effords

- Autoware's `autoware::component_interface_utils::NodeAdaptor` simplifies the ROS-API as well 
- [SMACC](https://github.com/robosoft-ai/SMACC) Proof on concept for reactive programming
- [RXROS](https://github.com/rosin-project/rxros2) Proof on concept for reactive programming
- [fuse](https://github.com/locusrobotics/fuse) Allows to model data flows, but it is focused on one application: sensor fusion. ICEY on the other hand is general 

