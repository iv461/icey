# ICEY 

A new, simple interface library for the Robot Operating System (ROS). 
In Icey, you can rapidly prototype Nodes in data-flow oriented manner: 

# Features 

The core idea is that in common robotics applications, almost everything published is a state: The pose of the robot, state of joints, initialization states, etc. 

In Icey, you can have states that are subscribed (`SubscribedState`), meaning they originate from other nodes. Or, states can be published, (`PublishedState`), meaning that they are held by the current node.

```cpp
auto current_velocity = icey::PublishedState<float>("current_velocity");

icey::spawn("ppc_controller_node");
```

## Parameters 

Paramters are persisted, but updates are communicated similar to topics. That is why they are very similar to states:

```cpp
auto max_velocity_parameter = icey::ParameterState<float>("maximum_velocity");
```


## Mixing with old ROS 2 API: 

This is a library that is a thin wrapper around ROS 2 (ROS 1 is also supported). 
It has virtually no buy-in cost: You can always still use callbacks for some topics, mix it with the regular ROS 2 API. 

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

## Features 

-  Which values to take for a node that has multiple inputs, but only one input changes ?
* Every input may provide a frequency interpolator 
* A strategy chooses between "last message" or interpolate
* in case of interpolate, the queue must be long, in case of last message, the queue can be 1

- `onCleanup` function to be able to call cleanup of external libraries, e.g. ONNX runtime (that would normally be the Node's destructor)

- Support Callback groups ? 
- A way to enable/disable the node 

- Maybe `RemindingPublishedState` that publishes regardless of whether the message changed or not

## Documentation 

- Document everything in a tutorial-style like modern JS frameworks (Svelte, Solid.js) do, otherwise adoptability will equal to zero 


## Examples 

Add examples for common applications: 

- Kalman filter: Multiple inputs, interpolate 
- SLAM: Front-end and backend, running asynchronously
- Controller like PPC, gets position and velocity, etc. 

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

