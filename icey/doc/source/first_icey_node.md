# Your first ICEY-Node 


In the following, we will assume you are already familiar writing ROS nodes in C++ (See [Official Client library Tutorials](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries.html)).

The ICEY library has two node classes: `icey::Node` and an `icey::LifecycleNode`. 
To create new nodes, you use the `icey::create_node<NodeType>(argc, argv, <node_name>)` function (default `NodeType` is `icey::Node`). This function simply creates a node with `std::make_shared<NodeType>`, but calls `rclcpp::init` beforehand if needed, so that you don't have to do it.

## Signal generator example

The signal generator example implements a sine signal generator. 

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
ICEY represents ROS primitives such as timers as a `Stream`, an abstraction over an asynchronous sequence of values. Streams have a method `.then` that registers a callback on each new value and returns a new stream. 

If you are familiar with JavaScript, this is essentially a promise, except that the state transitions are not final.

Such streams allow calls to `publish', i.e. they can be published directly. 
You do not need to create a publisher first, you just declare that the result should be published to a topic. 
Finally, we do not need to store the timer object anywhere, because the lifetime of entities in ICEY is bound to the lifetime of the node. This is generally true for other entities as well: In ICEY, you do not need to store subscriptions/timers/services as members of the class, ICEY does this bookkeeping for you.

## The ICEY-node 

The ICEY library has two node classes: `icey::Node` and `icey::LifecycleNode`.
The `icey::Node` is a subclass of an `rclcpp::Node`, so that you can drop-in replace you existing Node and gradually adopt ICEY's features.
ICEY offers all it's features through the `icey::Context` interface, accessed through `node->icey()`.

### Creating ICEY-nodes 

To create new ICEY-nodes, you use the `icey::create_node<NodeType>(argc, argv, <node_name>)` function (default `NodeType` is `icey::Node`). This function simply creates a node with `std::make_shared<NodeType>`, but calls `rclcpp::init` beforehand if needed, so that you don't have to do it.
This means, you do not have to use the `icey::create_node` function, you can instead create a node like you would create a regular `rclcpp::Node`.

Since the ICEY-nodes are just subclasses of regular nodes, you can also build them into shared libraries and load them at runtime, i.e. use them as *components* with the `RCLCPP_COMPONENTS_REGISTER_NODE`-macro.

### Spinning ICEY-nodes 

To spin ICEY-nodes, you use the `icey::spin_node(<node>)` function. This function simply spins the node in a `rclcpp::SingleThreadedExecutor`, but additionally calls `rclcpp::shutdown` at the end, so that you don't have to do it. 
This means, you do not have to use the `icey::spin_node(<node>)` function, you can instead spin it manually. 

```{warning}
ICEY-nodes can only be used with a single-threaded executor.
```


In the following, we will look more closely into how Subscriptions and Timers follow the `Stream` concept and how this changes the way of asynchronous programming. 
