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
[Service call example](icey_examples/src/service_client_async_await.cpp)
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
/// Here you declare in a single struct all parameters of the node:
struct NodeParameters {
  /// We can have regular fields :
  double amplitude{3};

  /// And as well parameters with constraints and a description:
  icey::Parameter<double> frequency{10., icey::Interval(0., 25.),
                                       std::string("The frequency of the sine")};
  /// We can also have nested structs with more parameters, they will be named others.max_amp, others.cov:
  struct OtherParams {
    double max_amp = 6.;
    std::vector<double> cov;
  } others;
};

auto node = icey::create_node<icey::Node>(argc, argv, "parameters_struct_example");
  /// Now create an object of the node-parameters that will be updated:
NodeParameters params;
node->icey().declare_parameter_struct(params, [&](const std::string &changed_parameter) {
    RCLCPP_INFO_STREAM(node->get_logger(),
                        "Parameter " << changed_parameter << " changed");
    });
```
# Dependencies: 

- C++20 
- ROS 2 Humble or Jazzy
- Boost (Hana)
- FMT

Note that ROS 2 Humble supports building with C++20 (`-std=c++20`) only recently (around mid-2024): Many fixes have been merged and then backported to Humble. The information you will find online that ROS 2 does not support C++20 is outdated. 

# Install 

Just clone this repository to your workspace, install dependencies and compile: 

```sh
git clone https://github.com/DriverlessMobility/icey.git
sudo apt install libboost-dev libfmt-dev
MAKEFLAGS="-j4" colcon build --packages-select icey icey_examples --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Note: Use `MAKEFLAGS="-j4"` to prevent you system from freezing. 

<details>

<summary>Explaination</summary>

By default, `colcon` will start as many compiler processes as there are CPU cores, if there are enough translation units (TU) to compile in parallel. The `icey_example` package contains ~20 examples and therefore TUs. Since GCC requires 1-3 GiB of RAM to compile a single TU using icey, on a machine with 20 CPU cores (such as a 12th generation Intel i7) and only 32 GiB of RAM, this will require more RAM than is available. So Linux starts swapping, which takes a very long time because at the same time the CPU load is also high. The result is an effectively unresponsive system.
Linux has an out-of-memory killer (OOM killer), but by default it is configured to be effectively useless, it won't kill the GCC processes.
By passing the option `MAKEFLAGS="-j4"`, only four jobs will be used, i.e. only 4 TUs will be compiled in parallel. This will prevent your system from freezing assuming you have at least 12 GiB of RAM.
Of course (and after you read this far) you can set it to whatever value you like.
We just want to prevent your first experience with ICEY from being "it freezes your system and you have to reboot", which would be very unpleasant.
</details>

# Documentation 

The documentation can be found here: TODO link 

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
- Memory strategy is not implemented
- Sub-nodes are not supported

# Features coming soon:

- Python support 
- Actions 

# Related effords

- Autoware's `autoware::component_interface_utils::NodeAdaptor` simplifies the ROS-API as well 
- [SMACC](https://github.com/robosoft-ai/SMACC) Proof on concept for reactive programming
- [RXROS](https://github.com/rosin-project/rxros2) Proof on concept for reactive programming
- [fuse](https://github.com/locusrobotics/fuse) Allows to model data flows, but it is focused on one application: sensor fusion. ICEY on the other hand is general 
- [r2r](https://github.com/m-dahl/r2r_minimal_node/blob/master/r2r_minimal_node/src/main.rs) Rust wrapper for ROS 2, at parts surprisingly similar since it uses tokio (an asynchronous programming library for Rust)

