# Your first ICEY-Node 

In the following, we will assume you are already familiar writing ROS nodes in C++ (See [Official Client library Tutorials](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries.html)).

## Install 

ICEY comes as a regular ROS 2 package, to install it just clone it in you colcon workspace and build it:

```sh
sudo apt install libboost-dev libfmt-dev
MAKEFLAGS="-j4" colcon build --packages-select icey icey_examples --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Using ICEY in an existing node (gradual adoption)

To use ICEY in an existing Node class, you simply add an `icey::Context` to it and initialize it in the constructor: 

```cpp
#include <icey/icey.hpp>

class MyNode: public rclcpp::Node {

  MyNode() : rclcpp::Node("my_node") {
    icey_context_ = std::make_shared<icey::Context>(this);
  }

  std::shared_ptr<icey::Context> icey_context_;  
};
```

## Convenience classes: `icey::Node` and `icey::LifecycleNode`

Alternatively, ICEY offers the convenience classes `icey::Node` and `icey::LifecycleNode`
that provide the `icey::Context` through the `icey()` method. 
They subclass `rclcpp::Node` and `rclcpp_lifecycle::LifecycleNode` respectively, so that they are drop-in replacements.

## First example 

```cpp
#include <icey/icey.hpp>

class MyNode : public rclcpp::Node {
public:
  MyNode() : rclcpp::Node("icey_context_example_node") {
    icey_context_ = std::make_shared<icey::Context>(this);
    icey_context_->create_timer(500ms, [this](size_t ticks) { on_tick(ticks); });
  }

  void on_tick(size_t ticks) {
    RCLCPP_INFO_STREAM(get_logger(), "Timer ticked: " << ticks);
  }

  std::shared_ptr<icey::Context> icey_context_;  
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::executors::MultiThreadedExecutor exec{rclcpp::ExecutorOptions(), 8};
  exec.add_node(node->get_node_base_interface());
  exec.spin();
}
```

See also the [Using ICEY Context](../../../icey_examples/src/using_icey_context.cpp) example.

In this example, we can already see some interesting things:
ICEY represents ROS primitives such as timers as a `Stream`, an abstraction over an asynchronous sequence of values. Streams have a method `.then` that registers a callback on each new value and returns a new stream. 

We also do not need to store the timer object anywhere, because the lifetime of entities in ICEY is bound to the lifetime of the node. In ICEY, you do not need to store subscriptions/timers/services as members of the class, ICEY does this bookkeeping for you.


```{warning}
ICEY-nodes can currently only be used with a single-threaded executor.
```

In the following, we will look more closely into how Subscriptions and Timers follow the `Stream` concept and how this changes the way of asynchronous programming. 
