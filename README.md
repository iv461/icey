# ICEY 

ICEY is a a new API for the Robot Operating System (ROS) 2 that allows for modern asynchronous programming using Streams, Promises and C++20 coroutines with async/await syntax. This simplifies application code and makes the asynchronous data-flow clearly visible. This enables fast prototyping with less boilerplate code.

Problems ICEY solves: 
  - Modern async/await syntax: All callbacks can be asynchronous (i.e. coroutines), you can `co_await` other asynchronous operations like service calls inside 
  - No danger of deadlocks since no manual spinning of the event loop is needed anymore
  - Consistend awaiting of asynchronous operations: service call, TF lookup  

It is fully compatible to the ROS 2 API and allows for gradual adoption since the `icey::Node` extends a regular ROS-Node. It supports all major features of ROS: parameters, subscriptions, publishers, timers, services, clients, TF. It supports not only regular nodes but also lifecycle nodes with a single API. 

ICEY operates smoothly together with the  `message_filters` package, and it uses it for synchronization. ICEY also allows for extension, demonstrated by the the support for `image_transport` camera subscription/publishers that is already implemented.

It offers additional goodies such as:
- Automatic bookkeeping of publishers/subscriptions/timers so that you do not have to do it 
- No callback groups needed for preventing deadlocks -- async/await allows for synchronously looking code while the service calls remain asynchronous
- Handle many parameters easily with a single parameter struct that is registered automatically using static reflection, so that you do not need to repeat yourself

ICEY supports ROS 2 Humble and ROS 2 Jazzy.

The [icey_examples](../../icey_examples) package contains many different example nodes, demonstrating the capabilities of ICEY.


## Examples

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

### Awaiting service calls
```cpp
auto service = node->icey().create_client<ExampleService>("set_bool_service");

icey().create_timer(1s)
    .then([this](size_t) -> icey::Promise<void> {
        /// Build a request each time the timer ticks
        auto request = std::make_shared<ExampleService::Request>();
        request->data = true;

        /// Call the service (asynchronously) and await the response with 1 second timeout:
        icey::Result<Response, std::string> result = co_await service.call(request, 1s);
        
        if (result.has_error()) {
            /// Handle errors: (possibly "TIMEOUT" or "INTERRUPTED")
            RCLCPP_INFO_STREAM(node->get_logger(), "Got error: " << result.error());
        } else {
            RCLCPP_INFO_STREAM(node->get_logger(), "Got response: " << result.value()->success);
        }
        co_return;
    })
```
See also the [Service client](../../../icey_examples/src/service_client_async_await.cpp) example.

### Asynchronous service server callbacks: 

The novelty of ICEY is that we can also use *asynchronous* callbacks (i.e. coroutines) as service callbacks:

```cpp
/// Create a service client for an upstream service that is actually capable of answering the
/// request.
auto upstream_service_client =
      node->icey().create_client<ExampleService>("set_bool_service_upstream");

node->icey().create_service<ExampleService>(
      "set_bool_service", 
        /// An asynchronous callback (coroutine) returns a Promise<Response>:
        [&](auto request) -> icey::Promise<Response> {

        /// Call the upstream service with 1s timeout asynchronously:
        icey::Result<Response, std::string> upstream_result = co_await upstream_service_client.call(request, 1s);

        if (upstream_result.has_error()) {
          RCLCPP_INFO_STREAM(node->get_logger(),
                             "Upstream service returned error: " << upstream_result.error());
          /// Return nothing: This will simply not respond to the client, leading to a timeout
          co_return nullptr;
        } else {
          Response upstream_response = upstream_result.value();
          /// Respond to the client with the upstream server's response:
          co_return upstream_response;
        }
        
      });
```
See also the [Service server](../../../icey_examples/src/service_server_async_await.cpp) example.


### Parameter declaration: 
ICEY also simplifies the declaration of many parameters: (very similar to the `dynamic_reconfigure`-package from ROS 1):

[Parameter struct example](icey_examples/src/parameters_struct.cpp)
```cpp
/// Here you declare in a single struct all parameters of the node:
struct NodeParameters {
  /// We can have regular fields:
  double amplitude{3};

  /// And as well parameters with constraints and a description:
  icey::Parameter<double> frequency{10., icey::Interval(0., 25.),
                                    std::string("The frequency of the sine")};
  /// We can even have nested structs with more parameters, they will be named others.max_amp,
  /// others.cov:
  struct OtherParams {
    double max_amp = 6.;
    std::vector<double> cov;
  } others;

};

class MyNode : public icey::Node {
public: 
  MyNode(std::string name) : icey::Node(name) {
    /// Now simply declare the parameter struct and a callback that is called when any field updates:
    icey().declare_parameter_struct(params_, [this](const std::string &changed_parameter) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Parameter " << changed_parameter << " changed");
    });
  }

  /// Store the parameters as a class member: 
  NodeParameters params_;
};
```
# Dependencies: 

- C++20 
- ROS 2 Humble or Jazzy
- Boost (Hana, typeinfo)
- FMT

Note that ROS 2 Humble is as of now (April 2025) already forward compatible with C++20 (compiling ROS-headers with `-std=c++20`): Information you will find online stating the contrary is simply outdated.

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

- Not thread-safe: only the `SingleThreadedExecutor` is supported currently
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

