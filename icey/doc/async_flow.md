# Asynchronous data-flow in ICEY 

ICEY allows to express asynchronous data-flow very easily, which is very useful for implementing service calls:

## Chaining of service calls 

One typical use-case is calling service calls periodically by a timer:

```cpp
icey().create_timer(1s)
    .then([this](size_t) {
        /// Build a request each time the timer ticks
        auto request = std::make_shared<ExampleService::Request>();
        request->data = true;
        return request;
    })
    .call_service<ExampleService>("my_service")
    .then([](ExampleService::Response::SharedPtr response) {
        /// Builda  second request
        return std::make_shared<ExampleService::Request>();
    })
    .call_service<ExampleService>("my_service2")
    .except([](std::string error) {});
```

See also the [service_client](../../icey_examples/src/service_client.cpp) example.

This operation is asynchronous in ICEY and so no dead-locks can occur. Services can be called from any other Stream, for example synchronizers

## Synchronization

Topics in ICEY can be synchronized in a very simple way using a single `synchronize` function:
```cpp 
auto camera_image_sub = node->icey().create_subscription<sensor_msgs::msg::Image>("camera");
auto point_cloud_sub = node->icey().create_subscription<sensor_msgs::msg::PointCloud2>("point_cloud");

///
node->icey().synchronize(camera_image_sub, point_cloud_sub)
    .then([](sensor_msgs::msg::Image::SharedPtr img, 
             sensor_msgs::msg::PointCloud2::SharedPtr point_cloud) {

    });
```

See also the [automatic_synchronization](../../icey_examples/src/automatic_synchronization.cpp) example.

This method will synchronize both topics by approximately matching their header timestamps. For this, ICEY uses the `message_filters::Synchronizer` with the `ApproxTime` policy. 

TODO sync_with_reference 

## Control flow: Multiple inputs and multiple outputs


### Single input, multiple output
You may wonder how you can express the control flow of publishing multiple times inside a callback:
```cpp 
auto first_publisher = node->create_publisher<Out1>("output_topic1", 1);
auto second_publisher = node->create_publisher<Out2>("output_topic1", 1);
auto sub = node->create_subscription<Msg>("topic", 1, 
    [](Msg::SharedPtr input) {

        auto output_msg1 = do_computation(input);
        auto output_msg2 = do_another_computation(input);

        first_publisher->publish(output_msg1);
        second_publisher->publish(output_msg2);
    });
```

In ICEY, you can directly call `.publish` on publisher streams (which is needed anyway for writing hardware ROS-drivers):

```cpp 
    auto first_publisher = node->icey().create_publisher<Out1>("output_topic1", 1);
    auto second_publisher = node->icey().create_publisher<Out2>("output_topic1", 1);

    node->icey().create_subscription<Msg>("topic", 1)
        .then([&](Msg::SharedPtr input) {

            auto output_msg1 = do_computation(input);
            auto output_msg2 = do_another_computation(input);

            first_publisher.publish(output_msg1);
            second_publisher.publish(output_msg2);
        });
```

The previous solution was the *push-pattern*.
Another way to do this in ICEY is the *pull-pattern*: (this is how Streams work by default): 

```cpp 
    auto [output1, output2] = node->icey().create_subscription<Msg>("topic", 1)
        .then([](Msg::SharedPtr input) {

            auto output_msg1 = do_computation(input);
            auto output_msg2 = do_another_computation(input);
            return std::make_tuple(output_msg1, output_msg2);
        }).unpack();
    output1.publish("output_topic1");
    output2.publish("output_topic2");
```

### Multiple input, single output 

We sometimes need to call the same function from multiple subscriber callbacks: 

TODO show here any

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

Commonly, control flow includes to return early in a callback:

```cpp
void VelocitySmoother::inputCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // If message contains NaN or Inf, ignore
  if (!nav2_util::validateTwist(*msg)) {
    return;
  }
  
  /// Otherwise, continue processing the message
  double result = doCalculation(msg);
}
```

In ICEY, we can achieve this by returning an `std::optional<T>`: If there is value, it is propagated further. If not, then the next stage won't be called.

```cpp
    node->icey().create_subscription<geometry_msgs::msg::Twist>("twist_input")
        .then([](const geometry_msgs::msg::Twist::SharedPtr msg) {
            std::optional<geometry_msgs::msg::Twist::SharedPtr> maybe_result;
            // If message contains NaN or Inf, ignore
            if (!nav2_util::validateTwist(*msg)) {
                return maybe_result;
            }
            maybe_result = msg; 
            return maybe_result;
        })
        .then([](geometry_msgs::msg::Twist::SharedPtr msg) {
            /// This will only be called if the previous stage returned something
        });
```

If this filtering can be efficiently implemented in a single function, we can write this even shorter: 

```cpp
    node->icey().create_subscription<geometry_msgs::msg::Twist>("twist_input")
        .filter([](const geometry_msgs::msg::Twist::SharedPtr msg) { return nav2_util::validateTwist(*msg);})
        .then([](const geometry_msgs::msg::Twist::SharedPtr msg) {
            doCalculation(msg);
        });
```

Filtering is also useful for conditional publishing. 

## Error-handling 

Some streams in ICEY can have errors: A service call might fail, or a transform might not be available.
If no error occurs a *value* is returned, otherwise an error. We handle errors by 


TODO explain promises here 

## Where are the callback groups ? 

Callback groups were introduced in ROS 2 mostly for two things: Speeding up computation by allowing callbacks to be called by the executor from different threads in parallel, and avoid deadlocks [1]. 

In ICEY deadlocks are prevented effectively by using the abstraction of Promises and using async/await. 
This way, altough the code looks synchronous, it is fully asynchronous and it is not possible to spin the event queue while a task is being processed, i.e. during a callback.

TODO 

In this chapter, we explain how more complex data-flows can be modeled using ICEY: combining timers with services, synchronizing, obtaining transforms and synchronizing with fixed rates.

