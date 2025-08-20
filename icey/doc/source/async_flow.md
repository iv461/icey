# Async flow: Synchronizing and filtering Streams

ICEY allows to express asynchronous data-flow very easily by applying transformations on Streams, leading to easy-to-read declarative code. 
In the following we look at various transformations that we can apply on Streams like  synchronizing with other streams or removing unwanted values.

## Publish

You can publish the values of every Stream using `.publish(<topic>, <qos>={}, <options>={})`

```cpp
  node->icey().create_timer(100ms)
    .then([](size_t ticks) {
        /// This function gets called each time the timer ticks
        std_msgs::msg::Float32 msg;
        msg.data = std::sin(0.1 * ticks * 2 * M_PI);
        return msg;
    })
    .publish("sine_signal");
```
See also the [signal generator example](../../../icey_examples/src/signal_generator.cpp):

For this to work, a Stream must hold a ROS-message type. 

## Filter 

You can filter a Stream by calling `.filter()` using a function that returns false if the message should be filtered and true if the message should be passed through: 

```cpp
  node->icey().create_subscription<geometry_msgs::PoseStamped>("ego_pose")
    /// Filter (i.e. remove) messages that contain NaNs:
    .filter([](geometry_msgs::PoseStamped::SharedPtr pose_msg) -> bool {
        return !(std::isnan(pose_msg->pose.x) 
                  ||std::isnan(pose_msg->pose.y) 
                  || std::isnan(pose_msg->pose.z));
    })
    .then([](geometry_msgs::PoseStamped::SharedPtr pose_msg) {
      /// Here we receive only NaN-free messages for further processing
    });
```

## Synchronization 

ICEY provides an easy way to use the synchronizers from the `message_filters` package: For example, synchronizing a camera and a LiDAR point cloud is as simple as:

```cpp
auto camera_image = node->icey().create_subscription<sensor_msgs::msg::Image>("camera");
auto point_cloud = node->icey().create_subscription<sensor_msgs::msg::PointCloud2>("point_cloud");

/// Synchronize by approximately matching the header time stamps (queue_size=100):
icey::synchronize_approx_time(100, camera_image, point_cloud)
    .then([](sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::PointCloud2::SharedPtr) {

    });
```

See also the [synchronization example](../../../icey_examples/src/synchronization.cpp)

This method approximately matches the header timestamps using the `message_filters::Synchronizer` with the `ApproxTime` policy. 

## Error handling: `unwrap_or`

Streams can yield not only values but also errors. 
For some transformations, it is mandatory to handle an error first. We can handle possible errors by calling `unwrap_or` and providing a function that receives the potential error. This results in an error-free stream.

```cpp
icey::Stream<size_t, std::string> fallible_stream = node->icey().create_timer(100ms)
  .then([](size_t ticks) -> icey::Result<size_t, std::string> {
      if(ticks % 10) {
        return icey::Result<size_t, std::string>::Ok(ticks);
      } else {
        return icey::Result<size_t, std::string>::Err("An error occurred");
      }
  });

icey::Stream<size_t> error_free_stream = fallible_stream
  .unwrap_or([](std::string error) {

  });
```

One example of a transformation that requires error handling is the `icey::synchronize_approx_time` transformation that we used earlier.
Error handling is mandatory here because such transformations may yield new errors.
However, streams are statically typed on the error. This means a stream can only store one type of error.

## Timeout 

You can check whether a message is too old (by it's header stamp) and register a callback when a timeout occurs using `.timeout(<duration>)`:

```cpp
  node->icey().create_subscription<geometry_msgs::PoseStamped>("ego_pose")
    /// Expect that every pose message is at most 200ms old
    .timeout(100ms)
    .unwrap_or([&](auto current_time, auto msg_time, auto max_age) {
        auto msg_dt = (current_time - message_timestamp).seconds();
        RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, fmt::format(
            "Timeout occured, message is old: {} seconds old, maximum allowed is {} seconds",
            msg_dt, max_age));
    }) 
    .then([](geometry_msgs::PoseStamped::SharedPtr pose_msg) {
      /// Here we receive only NaN-free messages for further processing
    });
```

## Buffer 

You can buffer `N` values of a Stream, obtaining a new Stream that yields an array of exactly `N` elements using `.buffer(<N>)`: 
```cpp 
  node->icey().create_subscription<std_msgs::msg::String>("/messages")
    .buffer(5)
    .then([&](std::shared_ptr<std::vector<std_msgs::msg::String>> messages) { 
      RCLCPP_INFO_STREAM(node->get_logger(), "Received 5 messages: " << fmt::format());
    });
``` 

## Control flow: Multiple inputs and multiple outputs

### Single input, multiple output

You can return multiple values from a handler, this will yield a Stream with value of type tuple. 
Such tuple-Streams can be `.unpack()`ed, the result is a tuple of multiple Streams: 

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