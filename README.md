# ICEY 

A new, simple data-driven interface library for the Robot Operating System (ROS). 
In Icey, you can rapidly prototype Nodes in data-flow oriented manner.

# Features 

The real power in ICEY is that you can declare computations, that will  be published automatically when the input changes: 

[Signal generator example](examples/signal_generator.cpp)
```cpp
#include <icey/icey_ros2.hpp>
int main(int argc, char **argv) {

    auto frequency = icey::declare_parameter<double>("frequency", 10.); // Hz, i.e. 1/s
    auto amplitude = icey::declare_parameter<double>("amplitude", 2.);
    auto timer_signal = icey::create_timer(100ms);
    /// Add a callback for the timer:
    auto rectangle_sig = timer_signal->then([](size_t ticks) { 
        std::optional<float> result; 
        //  1/10 Frequency divider
        if(ticks % 10 == 0) {
            result = (ticks % 20 == 0) ? 1.f : 0.f;
        } // Otherwise publish nothing
        return result;
    })->publish("rectangle_signal"); // And publish the result

    /// Add another callback for the timer
    auto sine_signal = timer_signal->then([&](size_t ticks) {
        /// We can access parameters in callbacks using .value() because parameters are always initialized first.
        double y = amplitude->value() * std::sin((0.1 * ticks) / frequency->value() * 2 * M_PI);
        return y;
    })->publish("sine_generator");

    icey::spawn(argc, argv, "signal_generator_example"); 
}
```

You can build your own data-driven pipeline of computations:


The key in ICEY is to describe the data-flow declaratively and then 
This not only simplifies the code, but prevents dead-lock bugs.ICEY automatically analyzes the data-flow graph and asserts no loops are present, performs topological sorting, and determines how many callback-groups are needed.

ICEY is a thin wrapper around the public ROS 2 API, it does not reinvent anything or use private implementation details.

TODO more 

## Parameters 

Paramters are persisted, but updates are communicated similar to topics. That is why they are very similar to states:

```cpp
auto max_velocity_parameter = icey::create_parameter<float>("maximum_velocity");
```


## Mixing with old ROS 2 API: 

Icey is a thin wrapper around ROS 2/1 and you can always switch to using the regular ROS 2 API: 

TODO 

# Dependencies: 

- needs ROS 2 (tested on Humble)
- Boost (Hana, Graph Library)

# Build and test 

```sh
colcon build  --packages-select icey
colcon test --packages-select icey 
# Run test
build/icey./promise_test
```
# Why should I use ICEY: 

TODO more convincing 

If the examples did not yet convince you: 

- Because ICEY removes unccesesary boolerplate code
- It automatically computes the data-flow synchronously to the data it depents on: No more `received_x`-flags and asynchronously spinning in timers, waiting to something to arrive 
- 

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

