# ICEY 

ICEY is a a new API for the Robot Operating System (ROS) 2 that uses modern asynchronous programming with Streams and async/await syntax. It makes the asynchronous data-flow clearly visible and simplifies application code. It enables fast prototyping with less boilerplate code.

It is fully compatible to the ROS 2 API, it does not reinvent anything and supports all major features: parameters, subscribers, publishers, timers, services, clients, TF pub/sub. It supports not only regular nodes but also lifecyle nodes with a single API. 

ICEY operates smoothly together with the  `message_filters` package, and it uses it for synchronization. ICEY also allows for extention, demonstated by the already implemented support for `image_transport` camera subscriber/publishers.

It offers additional goodies such as:
- Automatic bookeeping of publishers/subscribers/timers so that you do not have to do it 
- No callback groups needed for preventing deadlocks -- service calls are always asynchronous
- Handle many parameters easily with a single parameter struct that is registered automatically using static reflection, so that you do not need to repeat yourself

ICEY supports ROS 2 Humble and ROS 2 Jazzy.

Currently support only C++, Python-support is coming soon. 

The [icey_examples](icey_examples) package contains over one dozen of different example nodes, demonstrating the capabilites of ICEY.

# Features 

The real power in ICEY is that you can declare computations, that will  be published automatically when the input changes: 

[Signal generator example](icey_examples/src_signal_generator.cpp)
```cpp
#include <icey/icey.hpp>
int main(int argc, char **argv) {
    auto node = icey::create_node(argc, argv, "signal_generator_example");
    node->icey().create_timer(100ms)
        .then([&](size_t ticks) {
            /// We can access parameters in callbacks using .value() because parameters are always initialized first.
            double y = std::sin(0.1 * ticks * 2 * M_PI);
            return y;
        })
        .publish("sine_generator");
    icey::spin(node);
}
```

Using Streams (promises), you can build your own data-driven pipeline of computations, for example sequencing service calls: 
[Service call example](icey_examples/src/service_client.cpp)
```cpp
node->icey().create_timer(1s)
    /// Build a request when the timer ticks
    .then([](size_t) {
        auto request = std::make_shared<ExampleService::Request>();
        request->data = true;
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
        ...
    })
    /// Here we catch timeout errors as well as unavailability of the service:
    .except([](const std::string& error_code) {
        RCLCPP_INFO_STREAM(icey::node->get_logger(), "Service got error: " << error_code);
    });
```     
This programming model is fully asynchronous and therefore there is danger of deadlocks when chaining multiple callbacks. 

## Parameter declaration: 
ICEY also simplifies the declaration of many parameters: (very similar to the `dynamic_reconfigure`-package from ROS 1):

[Parameter struct example](icey_examples/src/parameters_struct.cpp)
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

# Performance: 

TODO summarize 

The Streams implemented are generally very fast, they have a small (non-zero), but in practice neglible overhead compared to plain callbacks. 
To demonstrate this, we translated a typical node from the Autoware project with multiple subscribers/publishers and measured the performance with perf. 
The evaluation showed an overall latency increase of only X.X % and no significant increase of the latency variance (jitter). 
See the [Evaluation]-section for more details. 


# (small) limitations

We generally aim with ICEY to support everything that ROS also supports. 
Still, there are some small limitations: 

- Only the SingleThreadedExecutor is supported currently
- Memory strategy is not implemented, but could be easily
- Sub-nodes

# Dependencies: 

- ROS 2 Humble or Jazzy
- Boost (Hana)
- C++20 is required for the parameters struct feature and coroutine-support

# Related effords

- Autoware's `autoware::component_interface_utils::NodeAdaptor` simplifies the ROS-API as well 
- [SMACC](https://github.com/robosoft-ai/SMACC) Proof on concept for reactive programming
- [RXROS](https://github.com/rosin-project/rxros2) Proof on concept for reactive programming
- [fuse](https://github.com/locusrobotics/fuse) Allows to model data flows, but it is focused on one application: sensor fusion. ICEY on the other hand is general 
- [r2r](https://github.com/m-dahl/r2r_minimal_node/blob/master/r2r_minimal_node/src/main.rs) Rust wrapper for ROS 2, at parts surprisingly similar since it uses tokio (an asynchronous programming library for Rust)

