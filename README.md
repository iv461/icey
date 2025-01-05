# ICEY 

A new, simple interface library for the Robot Operating System (ROS). 
In Icey, you can rapidly prototype Nodes in data-flow oriented manner: 

# Features 

The core idea is that in common robotics applications, almost everything published is either a state or a signal: The pose of the robot is a state, similar to the state of algorithms (i.e. "initialized"). Sensors on the other hand yield signals: Cameras, yaw rates etc.

In Icey, signals roughly correspond to subscribers and states to publishers.
The real power in ICEY is that you can declare computations, that will  be published automatically when the input changes: 

[Example1](examples/simple.cpp)
```cpp
#include <icey/icey_ros2.hpp>
#include "std_msgs/msg/float32.hpp"

int main(int argc, char **argv) {
    auto current_velocity = icey::create_signal<std_msgs::msg::Float32>("current_velocity");

    auto result = icey::compute_based_on([](std_msgs::msg::Float32 new_velocity) {
            std_msgs::msg::Float32 out_msg;
            out_msg.data = 2. * new_velocity.data;
            return out_msg;
        },
        current_velocity);

    result->publish("new_velocity");

    icey::spawn(argc, argv, "ppc_controller_node"); /// Create and start node
}
```

You can build your own data-driven pipeline of computations.

TODO more 

## Parameters 

Paramters are persisted, but updates are communicated similar to topics. That is why they are very similar to states:

```cpp
auto max_velocity_parameter = icey::create_parameter<float>("maximum_velocity");
```


## Mixing with old ROS 2 API: 

Icey is a thin wrapper around ROS 2/1 and you can always switch to using the regular ROS 2 API: 

```cpp
icey::node
```

You can always listen on changes of a state, like a subscriber callback:

```cpp

auto max_velocity_parameter = icey::SubscribedState<float>("maximum_velocity");
max_velocity_parameter.on_change([](const auto &new_value) {
    ...
});

auto max_velocity_parameter = icey::ParameterState<float>("maximum_velocity");
max_velocity_parameter.on_change([](const auto &new_value) {
    ...
});
```

Likewise, you can publish a state by calling `set()`: 

```cpp
auto slip_angle_state = icey::create_state<float>("/states/slip_angle");
slip_angle_state.set(0.01f) /// This will get published
```

# Robustness 

This library is designed to be robust against common usage mistakes: It will detect problems like cyclic dependencies that would cause infinite update cycles. It enforces this by creating a Directed Acyclic Graph (DAG) and using topological sorting to ensure a well-defined order of updates. 

# Similar projects 

Some similar projects exist, but not quite close:

- [SMACC] https://github.com/robosoft-ai/SMACC
- [RXROS] https://github.com/rosin-project/rxros2

# References 

- [1] https://en.wikipedia.org/wiki/Reactive_programming 
- [2] https://cs.brown.edu/~sk/Publications/Papers/Published/ck-frtime/
- [3] https://en.wikipedia.org/wiki/Functional_reactive_programming
- [4] https://svelte.dev/tutorial/
- [5] https://www.youtube.com/watch?v=cELFZQAMdhQ
- [6] https://docs.ros.org/en/jazzy/p/rclcpp/generated/

