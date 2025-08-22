# Your first ICEY-Node 


In the following, we will assume you are already familiar writing ROS nodes in C++ (See [Official Client library Tutorials](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries.html)).

## Using ICEY in an existing node (gradual adoption)

TODO 


## Convenience classes: `icey::Node` and `icey::LifecycleNode`

TODO rewrite ! 

The ICEY library has two node classes: `icey::Node` and an `icey::LifecycleNode`. 

## Signal generator example

The signal generator example implements a sine signal generator. 

```cpp
#include <icey/icey.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<icey::Node>("sine_generator");

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
    rclcpp::spin(node);
}
```

See also the [signal generator example](../../../icey_examples/src/signal_generator.cpp).

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

```{warning}
ICEY-nodes can currently only be used with a single-threaded executor.
```


In the following, we will look more closely into how Subscriptions and Timers follow the `Stream` concept and how this changes the way of asynchronous programming. 
