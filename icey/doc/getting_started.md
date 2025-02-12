# Getting started

ICEY is a a new API for the Robot Operating System (ROS) 2 that allows for fast prototyping and eliminating boilerplate code by using modern asynchronous programming.
It makes the asynchronous data-flow clearly visible and simplifies application code.

It is fully compatible to the ROS 2 API, it does not reinvent anything and supports all major features: Parameter, Subscribers, Publishers, Timers, Services, Clients, TF pub/sub. It supports not only regular nodes but also lifecyle nodes with a single API. 

ICEY operates smoothly together with the  `message_filters` package, using it's synchronizers. ICEY also allows for extention, demonstated by the already implemented support for `image_transport` camera subscriber/publishers.

It offers additional goodies such as:
- Automatic bookeeping of publishers/subscribers/timers so that you do not have to do it 
- No callback groups needed for preventing deadlocks -- service calls are always asynchronous
- Handle many parameters easily with a single parameter struct that is registered automatically using static reflection, so that you do not need to repeat yourself

ICEY supports ROS 2 Humble and ROS 2 Jazzy.

# Install ICEY 

ICEY comes as a regular ROS 2 package, to install it just clone it in you colcon workspace and build it:

```sh
git clone git@github.com:iv461/icey.git
sudo apt install liboost-dev 
colcon build  --packages-up-to icey icey_examples -DCMAKE_BUILD_TYPE=Release
```

The `icey_examples` package contains over one dozen of different example nodes, demonstrating the capabilites of ICEY.

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

The signature of `create_timer` is `icey::create_timer(period_time, <use_wall_time>, <is_one_off_timer>)`, one-off timers can be implemented therefore as well.

## Using timers as reference signal for synchronization 

TODO impl
You can use timer signals as a reference point to bring multiple topics to the same frequency by simply adding a timer signal to the `ApproxTime` filter as an input source:


## Signal routing 

In the following, more advanced signal routing strategies are explained.

## Single input, multiple output 

TODO 

If you need to publish multiple times

```cpp

auto period_time = 100ms;
double frequency = 10;
double amplitude = 2.;
size_t period_counter = 0;
icey::create_timer(period_time, "sine_generator").then([&](const rclcpp::Timer &timer) {
    
    std::make_tuple()
}).publish("sine_generator");
```


## Stopping the data flow: optional return

One common control flow is to return early in a callback:

```cpp
void VelocitySmoother::inputCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // If message contains NaN or Inf, ignore
  if (!nav2_util::validateTwist(*msg)) {
    RCLCPP_ERROR(get_logger(), "Velocity message contains NaNs or Infs! Ignoring as invalid!");
    return;
  }
  
  /// Otherwise, continue to do stuff
  double result = doCalculation(msg);
}
```

In ICEY, we can achieve the exact same by returning an `std::optional<T>`: If there is value, it is propagated further. If not, then the next stage won't be called.

```cpp
auto twist_signal = icey::create_subscription<geometry_msgs::msg::Twist>("twist_input");

auto twist_calculation_result = twist_signal.then([] (const geometry_msgs::msg::Twist::SharedPtr msg) {
  std::optional<double> maybe_result;
  // If message contains NaN or Inf, ignore
  if (!nav2_util::validateTwist(*msg)) {
    RCLCPP_ERROR(get_logger(), "Velocity message contains NaNs or Infs! Ignoring as invalid!");
    return maybe_result;
  }
  
  /// Otherwise, continue to do stuff
  maybe_result = doCalculation(msg);
  return maybe_result;
});

twist_calculation_result.then([](double result) {
    /// This will only be called if the previous call returned something.
});
```

This is especially important for conditional publishing


## Where are the callback groups ? 

Callback groups were introduced in ROS 2 mostly for two things: Speeding up computation by allowing callbacks to be called by the executor from different threads in parallel, and avoid deadlocks [1]. 

In ICEY deadlocks are prevented effectively by using the abstraction of Promises and using async/await. 
This way, altough the code looks synchronous, it is fully asynchronous and it is not possible to spin the event queue while a task is being processed, i.e. during a callback.

## References 

- [1] https://discourse.ros.org/t/simplifying-how-to-declare-parameters-in-ros-2/33272
- [2] Promise proposal by William Woodall and async/await discussion: https://github.com/ros2/ros2_documentation/issues/901
- [3] Mozilla Promise implementation: https://firefox-source-docs.mozilla.org/xpcom/mozpromise.html
- [4] std::future lacks continuation, current (2024) state of the art: https://ikriv.com/blog/?p=4916
- [5] https://engineering.fb.com/2015/06/19/developer-tools/futures-for-c-11-at-facebook/
- [1] Tutorial on how to use callback groups in rclcpp https://discourse.ros.org/t/how-to-use-callback-groups-in-ros2/25255