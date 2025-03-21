

## Parameters 

Parameters are are declared in ICEY similar to regular ROS. They model however the Stream concept and allow therefore to subscribe for updates like regular subscribers:

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

### Validators 
ICEY allows to constraint parameters to certain values using validators: They can be an arbitrary function generally.  Some convineint validators are implemented as well: `Set`s, defined explicitly by a list of values, or defined by a minimum and maximum value, i.e. `Interval`s.

Set of values: 
```cpp
auto mode_param = node->icey().declare_parameter<std::string>("mode", "single",   icey::Set<std::string>({"single", "double", "pulse"}));
```

Interval:
```cpp
auto frequency = node->icey().declare_parameter<double>("frequency", 10., icey::Interval(0., 100.));  // Hz, i.e. 1/s
```

See also the [signal generator example](../../icey_examples/src/signal_generator.cpp). 

## Parameter structs 

ICEY offers an even more great way of declaring many parameters at once: parameter structs.
Using ICEY, you can put many parameters in a normal struct, adding constraints like minimum and maximum values to them, or even using nested structs with more parameters:

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

Store an instance of this parameter struct as a member (as you are used to) then then simply call `declare_parameter_struct`, ICEY declares automatically every parameter:

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

Whereas with other approaches, you would have to manually repeat each parameter declaration, with ICEY it is just a single call to `declare_parameter_struct`. 

ICEY uses for this static reflection -- since C++20 it became possible to use static reflection completly transparent to the user, i.e. no annotations of any kind (macros, typing out everything twice) are needed for a struct to be eligible for reflection.

ICEY will update the parameter struct automatically when any parameter changes. 

When you start the example node and inspect it's parameters you see: 

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