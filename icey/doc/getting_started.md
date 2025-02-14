# Getting started

ICEY is a a new API for the Robot Operating System (ROS) 2 that uses modern asynchronous programming with Streams and async/await syntax. It makes the asynchronous data-flow clearly visible and simplifies application code. It enables fast prototyping with less boilerplate code.

It is fully compatible to the ROS 2 API, it does not reinvent anything and supports all major features: parameters, subscribers, publishers, timers, services, clients, TF pub/sub. It supports not only regular nodes but also lifecyle nodes with a single API. 

ICEY operates smoothly together with the  `message_filters` package, and it uses it for synchronization. ICEY also allows for extention, demonstated by the already implemented support for `image_transport` camera subscriber/publishers.

It offers additional goodies such as:
- Automatic bookeeping of publishers/subscribers/timers so that you do not have to do it 
- No callback groups needed for preventing deadlocks -- service calls are always asynchronous
- Handle many parameters easily with a single parameter struct that is registered automatically using static reflection, so that you do not need to repeat yourself

ICEY supports ROS 2 Humble and ROS 2 Jazzy.

The [icey_examples](../../icey_examples) package contains over one dozen of different example nodes, demonstrating the capabilites of ICEY.

# Install ICEY 

ICEY comes as a regular ROS 2 package, to install it just clone it in you colcon workspace and build it:

TODO update link 

```sh
git clone git@github.com:iv461/icey.git
sudo apt install liboost-dev
colcon build  --packages-up-to icey icey_examples -DCMAKE_BUILD_TYPE=Release
```


# Your first ICEY-Node 

In the following, we will assume you are already faimiliar writing ROS nodes in C++: How to write subscribers, publishers, and using callbacks. 

The key difference between ROS 2 and ICEY is that you can chain functions that will be called after the callback of a subscriber/timer/service returns, which means:

```cpp
#include <icey/icey.hpp>
int main(int argc, char **argv) {
  auto node = icey::create_node(argc, argv, "sine_generator");

  node->icey().create_timer(100ms)
    .then([](size_t ticks) {
        /// This function gets called each time the timer ticks
        return std::sin(0.1 * ticks * 2 * M_PI);
    })
    /// The returned value is published on the topic "sine_signal" after the timer ticked.
    .publish("sine_signal");
    icey::spin(node);
}
```

See also the [signal generator example](../../icey_examples/src/signal_generator.cpp).

ICEY represents every ROS primitive (sub/pub etc.) as a `Stream`, an abstraction of an asynchronous sequence of values. 
If you are familiar with JavaScript, this is essentially a Promise, only that the state transitions are not final.

In this simple example we already see some interesting features: You do not need to create a publisher beforehand, instead you declare that the result should be published on a topic. 
Also, we do not store a timer object anywhere, ICEY stores it internally so that it does not get out of scope. This holds true for classes as well: In ICEY, you do not have to store 
subscribers/timers/publisher etc. as members of the class, ICEY does this bookkeeping for you. 

Subscibers are Streams as well, an we also can have as many `.then`s as we want:

```cpp
  auto car_pose = node->icey().create_subscription<geometry_msgs::msg::TwistStamped>("car_pose", 1);
  
  auto cos_theta_half = car_pose
	.then([&](const geometry_msgs::msg::TwistStamped::SharedPtr &msg) {
		return msg.twist.z;
	});
	
  car_pose
	.then([&](const auto &msg) { /// 'auto' is supported as well
		return msg.twist.position;
	})
	.publish("car_position");
```

## Parameters 

ICEY simplifies handling of parameters, we can declare them with:

```cpp
auto offset_param = node->icey().declare_parameter<double>("offset", 0.);
```

Where the argument `0.` is the default value and the argument `true` indicates this parameter is dynamic, i.e. it can be changed at runtime.

The variable `offset_param` is as everything else in ICEY a Stream as well, and we can therefore subscribe to updates of the parameters with `.then`:

```cpp
offset_param.then([&](const auto &new_value) {
	RCLCPP_INFO_STREAM(node->get_logger(), "Offset changed: " << new_value);
});
```
If you instead want to obtain the value, you can call `.value()`:

```cpp
RCLCPP_INFO_STREAM(node->get_logger(), "Initial offset: " << offset_param.value());
```
This inly works for parameters -- they always have initial values, which is generally not true for other Streams.

We can also contrain parameters, for example in an interval:
```cpp
auto offset_param = icey::parameter<double>("offset", 0., icey::Interval(0, 1));
```

Or to a set of values: 

```cpp
auto offset_param = icey::parameter<double>("offset", 0., icey::Set(0, 0.5, 1));
```

See also the [signal generator example](../../icey_examples/src/signal_generator.cpp). 

## Parameter structs 

Do you have many parameters ? 
You can declare a single struct with all the parameters: 

```cpp
struct NodeParameters {
  /// We can have regular fields :
  double amplitude{3};

  /// And as well parameters with constraints and a description:
  icey::Parameter<double> frequency{10., icey::Interval(0., 25.),
                                       std::string("The frequency of the sine")};
  
  icey::Parameter<std::string> mode{"single", icey::Set<std::string>({"single", "double", "pulse"})};
  /// We can also have nested structs with more parameters, they will be named others.max_amp, others.cov:
  struct OtherParams {
    double max_amp = 6.;
    std::vector<double> cov;
  } others;
};
```
And then let ICEY declare all of them automatically to ROS: 

```cpp
auto node = icey::create_node<icey::Node>(argc, argv, "parameters_struct_example");
  
  /// Now create an object of the node-parameters that will be updated:
  NodeParameters params;

  /// Now simply declare the parameter struct and a callback that is called when any field updates: 
  icey::declare_parameter_struct(node->icey(), params, [&](const std::string &changed_parameter) {
        RCLCPP_INFO_STREAM(node->get_logger(),
                           "Parameter " << changed_parameter << " changed");
  });
```

See the [parameter structs example](../../icey_examples/src/parameters_struct.cpp) for details.

## Timers 

Timers are also signals:

```cpp
auto my_timer = node->icey().create_timer(100ms);
```

You can think of them as sources similar to subscribers but with no information, publishing periodically. 
We can easily register a callback and use the timer like you usually would: 

```cpp
auto my_timer = node->icey().create_timer(100ms);

my_timer.then([](size_t ticks) {
    /// Do work
});
```

## References 

- [1] https://discourse.ros.org/t/simplifying-how-to-declare-parameters-in-ros-2/33272
- [2] Promise proposal by William Woodall and async/await discussion: https://github.com/ros2/ros2_documentation/issues/901
- [3] Mozilla Promise implementation: https://firefox-source-docs.mozilla.org/xpcom/mozpromise.html
- [4] std::future lacks continuation, current (2024) state of the art: https://ikriv.com/blog/?p=4916
- [5] https://engineering.fb.com/2015/06/19/developer-tools/futures-for-c-11-at-facebook/
- [1] Tutorial on how to use callback groups in rclcpp https://discourse.ros.org/t/how-to-use-callback-groups-in-ros2/25255