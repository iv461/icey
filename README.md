# ICEY 

A new, simple interface library for the Robot Operating System (ROS). 
In Icey, you can rapidly prototype Nodes in data-flow oriented manner: 

# Features 

The core idea is that in common robotics applications, almost everything published is either a state or a signal: The pose of the robot is a state, similar to the state of algorithms (i.e. "initialized"). Sensors on the other hand yield signals: Cameras, yaw rates etc.

In Icey, signals roughly correspond to subscribers and states to publishers.
The real power comes in Icey that you can simply declare a data-driven pipeline of computations:

```cpp
#include <icey/icey_ros2.hpp>

int main(int argc, char **argv) {
    auto current_velocity = icey::create_signal<float>("current_velocity");

    icey::spawn(argc, argv, "ppc_controller_node"); /// Create and start node

}
```

## Parameters 

Paramters are persisted, but updates are communicated similar to topics. That is why they are very similar to states:

```cpp
auto max_velocity_parameter = icey::create_parameter<float>("maximum_velocity");
```


## Mixing with old ROS 2 API: 

Icey is a thin wrapper around ROS 2/1 and you can always switch and also mix 
with the regular ROS-API. It has virtually no buy-in cost: You can always still use callbacks for some topics, mix it with the regular ROS 2 API. 

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

# TODO 

## Robustness/ Preventing glitches: 

- We have a concept of a state that is owned by a single node only, but how can we enforce that the state is only published by a single node ? 
- Forbid and detect creating new states in callback
- Forbid subscribing to the same topic that is published by the same node 
- Forbid and detect cycles in the DAG: React has a "cyclic dependency detected" thing
- Prevent getting the value from an observable, i.e. from the object SubscribedState,  it is read-only
- Disallow graph change after node is created. Detect it. Provide instead "Node enable" functionality if someone wants to disable a publisher/subscriber
- After we have a DAG, we may add topological sorting. But at first we need a useEffect hook that translates to a timer, otherwise we are not able to make the programm run w/o external events 
- Really think about memory leaks, they should not be possible !
- Make sure we are moving all lambdas so that the captured values are moved as well !
- Check reference counts on all shared ptr to understand whrere they are all incremented
- Allowing only readable and only writable nodes and publishing resulting nodes is enough to ensure DAG-propery. But is it expressive enough ?
- Allow movable ? Likely not
- Global variable: Check linkage issues regarding ODR: What happens if icey is included multiple times in the translation unit ? -> we likely need a context,

## Features 

### Must-have for 0.1

- TF Buffer 
- Services
- `onCleanup` function to be able to call cleanup of external libraries, e.g. ONNX runtime (that would normally be the Node's destructor)
- Allow accessing the node: For logging and for all other objects
- Different Sync-policies, first implement latest
-  Synch-policies ?
* Every input may provide a frequency interpolator 
* A strategy chooses between "last message" or interpolate
* in case of interpolate, the queue must be long, in case of last message, the queue can be 1

- Actions

### Nice-to-have

- Prevent having to use an arrow -> only because everything needs to be reference-counted: Wrap the smart-ptr inside an object, i.e. use PIMPL OR even better, reference-track every thing internally in the staged list until the node is created, then attach everything to the node (!). And then simply return a weak reference (const T &) to the underlying objects
- A way to enable/disable the node 
- Maybe `RemindingPublishedState` that publishes regardless of whether the message changed or not
- Maybe Simulink-style blocks, i.e. constant, step, function etc.

- Lifecycle Nodes ?

## Documentation 

- Document everything in a tutorial-style like modern JS frameworks (Svelte, Solid.js) do, otherwise adoptability will equal to zero 

## Bugs 

- [] Fix segfault on termination, cleanup properly

## Examples 

Add examples for common applications: 

- Kalman filter: Multiple inputs, interpolate 
- SLAM: Front-end and backend, running asynchronously
- Controller like PPC, gets position and velocity, etc. 

# Missing features
In this early version, some things are missing: 
- Mutli-threading executor is not supported, everything is single-threaded. Therefore, not Callback-groups
- 
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

