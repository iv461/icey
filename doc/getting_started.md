# Subscribing and publishing 
TODO 

# Parameters 

TODO update 
## Declare and use parameters 

ICEY tries as to make using parameters, as everything else, as simple as possible.
Parameters are declared with:

```cpp
auto offset_param = icey::parameter<float>("offset", 0., true);
```

Where the argument `0.` is the default value and the argument `true` indicates this parameter is dynamic, i.e. it can be changed at runtime.

If we cannot decide on meaningful default values for the parameters and instead require the user to always give the node this parameter, 


We can also constrain the values of each parameter: 

```cpp
auto offset_param = icey::parameter<float>("offset", 0., true, icey::Interval(0, 1));
```

We can also constraint it to a set of values: 

```cpp
auto offset_param = icey::parameter<float>("offset", 0., true, icey::Set(0, 0.5, 1));
```

The issue that parameters in ROS require much unnecessary boilerplate code is 
widely known [1], and a couple of different solution were proposed.

Some noteworthy are: 
- [PickNick’s code-generation solution based on YAML files](https://github.com/PickNikRobotics/generate_parameter_library)
- [AutoWare's JSON schemas](https://github.com/orgs/autowarefoundation/discussions/3433)

These approaches alleviate in my opinion the problem greatly and are well scalable. 
If there is interest in integrating any of these great libraries into ICEY, please open an GitHub issue.

ICEY implements a simpler approch, not based on external files, more akin to the great `dynamic_reconfigure` package that was present in ROS 1. We will see in the future how the official ROS 2 API will evolve and which approach will eventually become the standard way.

### Using parameters: 

Parameters can be used


Do you commonly use a pareters struct that contains all your algorithm parameters ? 

(https://github.com/ros-navigation/navigation2/blob/main/nav2_map_server/include/nav2_map_server/map_io.hpp#L31)

```cpp
struct LoadParameters
{
  std::string image_file_name;
  double resolution{0};
  std::vector<double> origin{0, 0, 0};
  double free_thresh;
  double occupied_thresh;
  MapMode mode;
  bool negate;
};
```

# Subscribers 

> [!NOTE]
> A note to the message type: In ICEY, you always receive in the callback a regular C++ reference. 
Under the hood, ICEY always receives to allow for zero-overhead message passing when using intra-process communication, it simply dereferences the shared pointer before passing it to the user callback. 


# Managing resources 

In ICEY, everything that you spawsn is managed internally, you do not have to store subscribers, publishers, timers etc. internally as class field like you have to in ROS. 

# Managing state 

We understand that in robotics applications, we need to keep state somewhere: initialized algorithm libraries etc. Generally, you should try to use only state that is necessary, and pay attention to not store state redundantly. With that said, in ICEY the callbacks can always access variables from the outside, they do not need to be pure functions. This means, you simply capture your state in the lambda variable:

# Using classes

TODO 
In ICEY, it is still possible to organize nodes in an object-oriented way with classes. This is discouraged for very simple nodes, since the functional style can do the same with fewer lines. 
But ICEY still offers a functional API which is needed for example for components. 


Other use cases for this API are if you need to spawn multiple nodes or do not want to use to have global variables (neded by ICEY for the context).

# Timers 

Timers are also signals:

```cpp
auto my_timer = icey::create_timer(100ms);
```

You can think of them as sources similar to subscribers but with no information, publishing periodically. 
We can easily register a callback and use the timer like you usually would: 

```cpp
auto my_timer = icey::create_timer(100ms);

my_timer->then([](size_t ticks) {
    /// Do work
});
```

## Using timers as signal generators

The power in ICEY comes from the fact that timers can be used as signal generators. For example, you can easily implement your own sine signal generator and publish it: 


```cpp
auto period_time = 100ms;
auto frequency = icey::declare_parameter<double>("frequency", 10.); // Hz, i.e. 1/s
auto amplitude = icey::declare_parameter<double>("amplitude", 2.);

auto timer_signal = icey::create_timer(period_time);

auto sine_signal = timer_signal->then([&](size_t ticks) {
    std_msgs::msg::Float32 float_val;
    double period_time_s = 0.1;
    /// We can .get() parameters since they are always initialized first, so at this point they are alreay there        
    double y = amplitude->get() * std::sin((period_time_s * ticks) / frequency->get() * 2 * M_PI);
    float_val.data = y;
    return float_val;
});

sine_signal->publish("sine_generator");
```

The signature of `create_timer` is `icey::create_timer(period_time, <use_wall_time>, <is_one_off_timer>)`, one-off timers can be implemented therefore as well.

## Using timers as reference signal for synchronization 

TODO impl
You can use timer signals as a reference point to bring multiple topics to the same frequency by simply adding a timer signal to the `ApproxTime` filter as an input source:


# Signal routing 

In the following, more advanced signal routing strategies are explained.

## Single input, multiple output 

TODO 

If you need to publish multiple times

```cpp

auto period_time = 100ms;
double frequency = 10;
double amplitude = 2.;
size_t period_counter = 0;
icey::create_timer(period_time, "sine_generator")->then([&](const rclcpp::Timer &timer) {
    
    std::make_tuple()
}).publish("sine_generator");
```


# Stopping the data flow: optional return

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

auto twist_calculation_result = twist_signal->then([] (const geometry_msgs::msg::Twist::SharedPtr msg) {
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

twist_calculation_result->then([](double result) {
    /// This will only be called if the previous call returned something.
});
```

This is especially important for conditional publishing

# References 

- [1] https://discourse.ros.org/t/simplifying-how-to-declare-parameters-in-ros-2/33272