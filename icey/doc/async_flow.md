# Asynchronous data-flow in ICEY 

ICEY allows to express asynchronous data-flow very easily, which is very usefull for implementing service calls:


## Chaining of service calls 

TODO 

One typical use-case is calling service calls periodically by a timer:

```cpp
icey().create_timer(1s)
    /// Build a request when the timer ticks
    .then([this](size_t) {
        auto request = std::make_shared<ExampleService::Request>();
        request->data = true;
        RCLCPP_INFO_STREAM(this->get_logger(),
                            "Timer ticked, sending request: " << request->data);
        return request;
    })
```

## Synchronization

You can use timer signals as a reference point to bring multiple topics to the same frequency by simply adding a timer signal to the `ApproxTime` filter as an input source:

In the following, more advanced signal routing strategies are explained.

## Control flow: Multiple inputs and multiple outputs


### Single input, multiple output
You may wonder how you can express the control flow of publishing multiple times inside a callback:
```cpp 
auto first_publisher = node->create_subscription<Out1>("output_topic1", 1);
auto second_publisher = node->create_subscription<Out2>("output_topic1", 1);
auto sub = node->create_subscription<Msg>("topic", 1, 
    [](Msg::SharedPtr input) {

        auto output1 = do_computation(input);
        auto output2 = do_another_computation(input);

        first_publisher->publish(output1);
        second_publisher->publish(output2);
    });
```

In ICEY, you can directly call `.publish` on publisher streams (which is needed anyway for writing hardware ROS-drivers):

```cpp 
    auto first_publisher = node->icey().create_subscription<Out1>("output_topic1", 1);
    auto second_publisher = node->icey().create_subscription<Out2>("output_topic1", 1);

    node->icey().create_subscription<Msg>("topic", 1)
        .then([&](Msg::SharedPtr input) {

            auto output1 = do_computation(input);
            auto output2 = do_another_computation(input);

            first_publisher.publish(output1);
            second_publisher.publish(output2);
        });
```

The previous solution was the *push-pattern*.
Another way to do this in ICEY is the *pull-pattern*: (this is how Streams work by default): 

```cpp 
    auto [output1, output2] = node->icey().create_subscription<Msg>("topic", 1)
        .then([](::SharedPtr input) {

            auto output1 = do_computation(input);
            auto output2 = do_another_computation(input);
            return std::make_tuple(output1, output2);
        }).unpack();
    output1.publish("output_topic1");
    output2.publish("output_topic2");
```

### Multiple input, single output 

We sometimes need to call the same function from multiple subscriber callbacks: 


```cpp 
    navsat_fix_sub_ = node->create_subscription<NavsatFix>("/navsat_fix", 1, 
        [](gps_msgs::NavSatFix::SharedPtr navsat_msg) {
            auto position = project(navsat_msg);
            on_gnss_position(position);
        });
    ins_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>("/gnss_ins_pose", 1, 
        [](geometry_msgs::msg::PoseStamped::SharedPtr gnss_pose) {
            on_gnss_position(gnss_pose->pose.position);
        });
``` 

We could of course register two callbacks
TODO finish with any filter

```cpp 
    auto navsat_fix = node->icey().create_subscription<NavsatFix>("/navsat_fix", 1);
    auto ins_pose = node->icey().create_subscription<geometry_msgs::msg::PoseStamped>("/gnss_ins_pose", 1);
``` 

## Cancellation (stopping the data-flow)

One common control flow is to return early in a callback:

```cpp
void VelocitySmoother::inputCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // If message contains NaN or Inf, ignore
  if (!nav2_util::validateTwist(*msg)) {
    RCLCPP_ERROR(get_logger(), "Velocity message contains NaNs or Infs! Ignoring as invalid!");
    return;
  }
  
  /// Otherwise, continue processing the message
  double result = doCalculation(msg);
}
```

In ICEY, we can achieve the exact same by returning an `std::optional<T>`: If there is value, it is propagated further. If not, then the next stage won't be called.

```cpp
    node->icey().create_subscription<geometry_msgs::msg::Twist>("twist_input")
        .then([](const geometry_msgs::msg::Twist::SharedPtr msg) {
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
```
This is especially important for conditional publishing based on a parameter for example. 

If this filtering can be efficiently implemented in a single function, we can write this even shorter: 

```cpp
    node->icey().create_subscription<geometry_msgs::msg::Twist>("twist_input")
        .filter([](const geometry_msgs::msg::Twist::SharedPtr msg) { return nav2_util::validateTwist(*msg);})
        .then([](const geometry_msgs::msg::Twist::SharedPtr msg) {
            doCalculation(msg);
        });
```

## Where are the callback groups ? 

Callback groups were introduced in ROS 2 mostly for two things: Speeding up computation by allowing callbacks to be called by the executor from different threads in parallel, and avoid deadlocks [1]. 

In ICEY deadlocks are prevented effectively by using the abstraction of Promises and using async/await. 
This way, altough the code looks synchronous, it is fully asynchronous and it is not possible to spin the event queue while a task is being processed, i.e. during a callback.

TODO 

In this chapter, we explain how more complex data-flows can be modeled using ICEY: combining timers with services, synchronizing, obtaining transforms and synchronizing with fixed rates.

