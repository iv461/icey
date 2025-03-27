# Parameters 

## Parameter structs 


### Motivation 

Typically, a ROS node has many parameters that require additional constraints such as minimum and maximum values. 
Many application developers create a struct that contains a copy of the current value of each ROS parameter. This parameter struct is then stored as a field of the node class.

If a ROS node wraps an algorithm that has many parameters, this means many calls to `declare_parameter`, `get_parameter`, and also every single parameter has to be copied to the parameter struct in the on-change callback. This means a lot of boilerplate code. 

ICEY provides a solution to this problem by automatically declaring all parameters of a given struct using static reflection.

With ICEY, you can put many parameters in a normal struct, add constraints like minimum and maximum values, or even use nested structs with more parameters:

```cpp
/// Here you declare in a single struct all parameters of the node:
struct NodeParameters {
  /// We can have regular fields:
  double amplitude{3};

  /// And as well parameters with constraints and a description:
  icey::Parameter<double> frequency{10., icey::Interval(0., 25.),
                                    std::string("The frequency of the sine")};

  /// Constrain a string parameter to a set of values:
  icey::Parameter<std::string> mode{"single",
                                    icey::Set<std::string>({"single", "double", "pulse"})};

  /// We can even have nested structs with more parameters, they will be named others.max_amp,
  /// others.cov:
  struct OtherParams {
    double max_amp = 6.;
    std::vector<double> cov;
  } others;

};
```

After having declared a parameter struct, store it as a member of the node class and then simply call `declare_parameter_struct`, ICEY declares automatically every parameter, without any further boilerplate code:

```cpp
class MyNode : public icey::Node {
public: 
  MyNode(std::string name) : icey::Node(name) {
    /// Now simply declare the parameter struct and a callback that is called when any field updates:
    icey().declare_parameter_struct(params_, [this](const std::string &changed_parameter) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Parameter " << changed_parameter << " changed");
    });
  }

  /// Store the parameters as a class member: 
  NodeParameters params_;
};

auto node = icey::create_node<MyNode>(argc, argv, "icey_parameters_struct_example");
```

See also the [parameter structs example](../../icey_examples/src/parameters_struct.cpp).

After examining the parameters of the sample node, you see:

```sh 
ros2 param dump /icey_parameters_struct_example

/icey_parameters_struct_example:
  ros__parameters:
    amplitude: 3.0
    frequency: 10.0
    mode: single
    others:
      cov: []
      max_amp: 6.0
      [...]
    use_sim_time: false
```

ICEY will update the parameter struct automatically when any parameter changes. 

Whereas with other approaches you would have to manually repeat each parameter declaration, with ICEY it is just a single call to `declare_parameter_struct'. 

ICEY uses static reflection for this -- since C++20 it is possible to use static reflection completely transparent to the user, i.e. no annotations of any kind (macros, typing everything twice) are required for a struct to be eligible for reflection.


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

See also the [signal generator example](../../icey_examples/src/signal_generator.cpp). 

## Declaring single parameters 

You can also declare single parameters. They are Streams and allow to subscribe for updates:

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
