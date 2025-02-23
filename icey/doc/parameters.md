

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
