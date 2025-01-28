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

The key in ICEY is to describe the data-flow declaratively and then 
This not only simplifies the code, but prevents dead-lock bugs.ICEY automatically analyzes the data-flow graph and asserts no loops are present, performs topological sorting, and determines how many callback-groups are needed.

ICEY is a thin wrapper around the public ROS 2 API, it does not reinvent anything or use private implementation details.

# Key features: 

- Because ICEY removes unccesesary boolerplate code
- Easy asynchronous programming using promises, you do not [have to deal with callback groups in order to prevent deadlocks](https://docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html) or spawning extra threads
- It automatically computes the data-flow synchronously to the data it depents on: No more `received_x`-flags and asynchronously spinning in timers, waiting to something to arrive 
- Efficiency: No additional dynamic memory allocations compared to plain ROS happen after the node is initialized, also not for error handling thanks to using Result-types


# Not supported yet

- Only the SingleThreadedExecutor is supported currently. That is mainly because the MultiThreadedExecturos does not have a properly implemented scheduler and instead sufferts from a [starvation](https://github.com/ros2/rclcpp/pull/2702) [issue](https://github.com/ros2/rclcpp/issues/2402). I have no intereset in making the code thread-safe as long as there is no reliable MT-executor anyway. Note that this does not mean you cannot use multiple threads for your computation-heavy algorihtms: You can still use OpenMP to parallelize them, only all the callbacks will be called from a single (the same) thread. 
- Memory strategy is not implemented, but could be easily, simply add the arguments everywhere 
- Sub-nodes

## Parameters 

Parameters are persisted, but updates are communicated similar to topics. That is why they are very similar to states:

```cpp
auto max_velocity_parameter = icey::create_parameter<float>("maximum_velocity");
```


## Mixing with old ROS 2 API: 

Icey is a thin wrapper around ROS 2/1 and you can always switch to using the regular ROS 2 API: 

TODO 

# Dependencies: 

- ROS 2 (tested on Humble)
- Boost Hana

# Build and test 

```sh
colcon build  --packages-up-to icey_examples
colcon test --packages-select icey 
# Run tests
build/icey./promise_test
```

# Robustness 

This library is designed to be robust against common usage mistakes: It will detect problems like cyclic dependencies that would cause infinite update cycles. It enforces this by creating a Directed Acyclic Graph (DAG) and using topological sorting to ensure a pre-determined order of updates that guarantees all predecessor nodes are updated before their children ("deterministic update")

# Related effords

- Autoware's `autoware::component_interface_utils::NodeAdaptor` simplifies the ROS-API as well 
- [SMACC](https://github.com/robosoft-ai/SMACC) Proof on concept
- [RXROS](https://github.com/rosin-project/rxros2) Proof on concept

## Not directly related
- [fuse](https://github.com/locusrobotics/fuse) Allows to model data flows, but it is focused on one application: sensor fusion. ICEY on the other hand is general 

# References 

- [1] https://en.wikipedia.org/wiki/Reactive_programming 
- [2] https://cs.brown.edu/~sk/Publications/Papers/Published/ck-frtime/
- [3] https://en.wikipedia.org/wiki/Functional_reactive_programming
- [4] https://svelte.dev/tutorial/
- [5] https://www.youtube.com/watch?v=cELFZQAMdhQ
- [6] https://docs.ros.org/en/jazzy/p/rclcpp/generated/

