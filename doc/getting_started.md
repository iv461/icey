# Subscribing and publishing 
TODO 

# Parameters 

## Declare and use parameters 

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

TODO 
A note to the message type: In ICEY, you always receive in the callback a regular C++ reference. 
Under the hood, ICEY always receives to allow for zero-overhead message passing when using intra-process communication, it simply dereferences the shared pointer before passing it to the user callback. 


# Managing resources 

In ICEY, everything that you spawsn is managed internally, you do not have to store subscribers, publishers, timers etc. internally as class field like you have to in ROS. 

# Managing state 

We understand that in robotics applications, we need to keep state somewhere: initialized algorithm libraries etc. Generally, you should try to use only state that is necessary, and pay attention to not store state redundantly. With that said, in ICEY the callbacks can always access variables from the outside, they do not need to be pure functions. This means, you simply capture your state in the lambda variable:

## Cleaning up the node 

In ICEY, you can declare a clean-up function that is executed when the node is destruced, very similar to a custom destructor: 

```cpp
icey::spawn(argc, argv, "onnx_node").cleanup([]() {
        /// Cleanup ONNX
}); /// Create and start node
```

```cpp
auto my_timer = icey::create_timer(100ms, "my_timer1");
```


# Using classes

TODO 
In ICEY, it is still possible to organize nodes in an object-oriented way with classes. This is discouraged for very simple nodes, since the functional style can do the same with fewer lines. 
But ICEY still offers a functional API which is needed for example for components. 


Other use cases for this API are if you need to spawn multiple nodes or do not want to use to have global variables (neded by ICEY for the context).

# Timers 

Timers are also signals:

```cpp
auto my_timer = icey::create_timer(100ms, "my_timer1");
```

You can think of them as sources similar to subscribers but with no information, publishing periodically. 
We can easily register a callback and use the timer like you usually would: 

```cpp
auto my_timer = icey::create_timer(100ms, "my_timer1");

my_timer.then([](const rclcpp::Timer &timer) {
    /// Do work, implement one-off timer by cancelling it here with timer.cancel() etc.
});
```

## Using timers as signal generators

The power in ICEY comes from the fact that timers can be used as signal generators. For example, you can easily implement your own sine signal generator and publish it: 

TODO check formula, polish API, publish message etc.

```cpp

auto period_time = 100ms;
double frequency = 10;
double amplitude = 2.;
size_t period_counter = 0;
icey::create_timer(period_time, "sine_generator").then([&](const rclcpp::Timer &timer) {
    double y = amplitude * std::sin((period_time * period_counter) / frequency * 2 * M_PI);
    period_counter++;
    return y;
}).publish("sine_generator");
```


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
icey::create_timer(period_time, "sine_generator").then([&](const rclcpp::Timer &timer) {
    
    std::make_tuple()
}).publish("sine_generator");
```


### Conditional publishing 

TODO it is important we support conditional publishing, for example debug publishers based on a parameter.