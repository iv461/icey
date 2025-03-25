# Asynchronous data-flow in ICEY 

ICEY allows to express asynchronous data-flow very easily by applying transformations on Streams, leading to easy-to-read declarative code. 
In the following we look at various transformations that we can apply on Streams. 

## Motivation: avoiding *callback hell*: 

When writing complex ROS application code that uses in combination subscribers, timers and services, the limits of the regular ROS API quickly become apparent.

One typical use-case is calling service calls periodically by a timer:
Many ROS application developers are asking how to do synchronous service calls. In ROS 2, there is no synchronous service call anymore (compared to ROS 1), making the translation of ROS 1 code challenging. But actually, the motivation behind asking for a synchronous service call is that we want to do something only after the service call returned the response. Meaning, we want to enforce a sequence of the operations. 
This is actually possible already in regular ROS 2 by simply nesting the callbacks:

```cpp
/// Inside the callback of a timer, we want to call a service and then wait for a transform: 
void on_timer() {
    auto req = create_request();
    ...
    client->async_send_request(req, [](auto future) {
        if(!future.valid()) {
            /// do error handling
            return;
        }
        client1->async_send_request(req, [](auto future1) {
            if(!future1.valid()) {
                /// do error handling
                return;
            }
            tf_buffer->waitForTransform(target, source, [](auto tf_future) {
                if(tf_future.valid()) {
                    auto transform = tf_future.get().second;
                        transform_message(msg, transform);
                    ...
                }
            }); 
        });    
    });
}
```
By nesting the callbacks, you can achieve the sequence of operations. 
But as you see, nesting many callbacks (in this case 3) starts making the code unreadable due to excessive indentation, a common problem that is called *callback hell*. 

## Chaining of service calls 

One typical use-case is calling service calls periodically by a timer:

```cpp
auto service = node->icey().create_client<ExampleService>("set_bool_service");

icey().create_timer(1s)
    .then([this](size_t) -> icey::Promise<void> {
        /// Build a request each time the timer ticks
        auto request = std::make_shared<ExampleService::Request>();
        request->data = true;
        
        icey::Result<Response, std::string> result = co_await service.call(request, 1s);
        if (result.has_error()) {
            /// Handle errors: (possibly "TIMEOUT" or "INTERRUPTED")
            RCLCPP_INFO_STREAM(node->get_logger(), "Got error: " << result.error());
        } else {
            RCLCPP_INFO_STREAM(node->get_logger(), "Got response: " << result.value()->success);
        }
        co_return;
    })
```

The biggest difference of ICEY to regular ROS is that we can perform asynchronous operations inside callbacks: We call the service in the timer callback, and due using C++20 coroutines, we can co_await the service response without blocking the event queue. And all this while using only a single thread.

Asynchronous operations can be called from any callback an no deadlocks can occur. 

See also the [service_client](../../icey_examples/src/service_client_async_await.cpp) example.

No more dead-locks can occur. Services can be called from any other Stream, for example synchronizers


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

Streams in ICEY can have errors: A service call might fail, or a transform might not be available.
If no error occurs a *value* is returned, otherwise an error. We handle errors by creatin


## Where are the callback groups ? 

Callback groups were introduced in ROS 2 mostly for two things: Speeding up computation by allowing callbacks to be called by the executor from different threads in parallel, and avoid deadlocks [1]. 

In ICEY deadlocks are prevented effectively by using the abstraction of Promises and using async/await. 
This way, altough the code looks synchronous, it is fully asynchronous and it is not possible to spin the event queue while a task is being processed, i.e. during a callback.

TODO 

In this chapter, we explain how more complex data-flows can be modeled using ICEY: combining timers with services, synchronizing, obtaining transforms and synchronizing with fixed rates.

