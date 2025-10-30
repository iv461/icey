# Parameters 

### Validators 

ICEY allows to constraint parameters to certain values using validators: They can be an arbitrary functions generally.  Some convenient validators are implemented as well: `Set`s, defined explicitly by a list of values, or defined by a minimum and maximum value, i.e. `Interval`s.

Set of values: 
```cpp
auto mode_param = node->icey().declare_parameter<std::string>("mode", "single",   icey::Set<std::string>({"single", "double", "pulse"}));
```

Interval:
```cpp
auto frequency = node->icey().declare_parameter<double>("frequency", 10., icey::Interval(0., 100.));  // Hz, i.e. 1/s
```

See also the [signal generator example](../../../icey_examples/src/signal_generator.cpp). 

## Declaring single parameters 

You can also declare single parameters. Compared to regular ROS, the API is simplified as requested by some users [1]. Parameters are Streams and allow to subscribe for updates:

```cpp
bool is_read_only = false;
bool ignore_overrides = false;
double default_value = 0.;
auto offset_param = node->icey().declare_parameter<double>("offset", default_value, icey::Validator<double>{}, "description", is_read_only, ignore_overrides);
```

The parameter name and the default value are mandatory, all other arguments are not. 

You can subscribe to updates of parameters with `.then`:

```cpp
offset_param.then([&](const auto &new_value) {
	RCLCPP_INFO_STREAM(node->get_logger(), "Offset changed: " << new_value);
});
```

If you instead want to obtain the value, you call `.value()`:

```cpp
RCLCPP_INFO_STREAM(node->get_logger(), "Initial offset: " << offset_param.value());
```

This only works for parameters -- they always have initial values, which is generally not true for other Streams.

## References 

- [1] A user requesting a simpler ROS API on ROS discourse: [https://discourse.ros.org/t/simplifying-how-to-declare-parameters-in-ros-2/33272](https://discourse.ros.org/t/simplifying-how-to-declare-parameters-in-ros-2/33272)
