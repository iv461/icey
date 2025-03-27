# Publish and subscribe

In the following, we will look at how to create publishers and subscriptions.

## The Stream concept 

Streams are a fundamental concept upon which ICEY is built. An `icey::stream` is an asynchronous abstraction that represents a sequence of values, potentially infinitely many. 
Subscribers, service servers, and timers all model the stream concept. 
You can either register a callback on a stream that's invoked when a new value (i.e. ROS message) arrives, or you can asynchronously wait for a new value using `co_await stream`. 

Streams shine when we apply transformations to them -- synchronization, filtering, buffering, we can also just `publish()` a stream. More on this later.


## Subscribe

Subscriptions are created using `node->icey().create_subscription<Message>(<topic-name>, <qos>, <callback>={}, <options>={})`. The meaning of the arguments is the same as that to the `create_subscription`-function of a regular ROS-node. 

Example: 

```cpp
auto node = icey::create_node(argc, argv, "yolo_node");
icey::SubscriptionStream<sensor_msgs::msg::Image> camera_image = node->icey().create_subscription<sensor_msgs::msg::Image>("camera", 
  [](sensor_msgs::msg::Image::SharedPtr msg) {
      /// Consume camera message here ..
  },
  rclcpp::SensorDataQoS());
```

Key differences to regular ROS are: 
  - `create_subscriber` returns a `Stream` (you can access the `rclcpp::Subscriber` using `stream.subscriber`)
  - Asynchronous functions, i.e. coroutines, can be used as callbacks
  - The lifetime of the subscription is bound to the lifetime of the node: This means, you don't need to store  the subscriber as a member in the node class (i.e. do bookkeeping)
  - The quality of service is optional: If it is not given, the so called *system default* one is used
  - The callback is optional: You do not need to provide a callback if you plan for example to first synchronize the subscriber with another stream

## Publishers

Publishers are created using `node->icey().create_publisher<Message>(<topic-name>, <qos>, <options>={})`. The meaning of the arguments is the same as that to the `create_publisher`-function of a regular ROS-node. 

Example: 

```cpp
auto node = icey::create_node(argc, argv, "yolo_node");
icey::PublisherStream<sensor_msgs::msg::Image> publisher = node->icey().create_publisher<sensor_msgs::msg::Image>("camera", rclcpp::SensorDataQoS());

publisher.publish(message);
```

Key differences to regular ROS are: 
  - The lifetime of the publisher is bound to the lifetime of the node: You do not have to store the subscriber in the node class, i.e. do bookkeeping, ICEY does this for you
  - `create_publisher` returns a `stream, but you can access the ROS-publisher using `steam.publisher` 
  - If no quality of service is given, a uses the so called *system default*  

### Awaiting `publish`

Publishing a message with reliable quality of service involves two steps: sending the message and waiting for an acknowledgment (ACK) from the receiver. The actual sending is a synchronous operation, but the arrival of the ACK is asynchronous: we don't know when (or if) we will receive the ACK from another node. 
This means that `publish' with reliable quality of service is actually an asynchronous function. 
Calling `publish` is very similar to calling a service in terms of the sequence of operations. 
ROS treats publishing a message as a fire-and-forget task: The `publish` call sends the message, but does not wait for the ACK to be received, and returns immediately after the message is sent.

What we want therefore is a publish function that returns a promise and allows to be awaited. By calling `co_await publish(<message>)`, we suspend execution until the has ACK arrived.
This also allows to have the old behavior: We simply do not need to `co_await` if it is undesired. 
This is how many other message passing libraries implement a `publish` function [1, 2, 3].

Unfortunately, ROS does not provide an API for asynchronous ACK waiting, only a synchronous one (`rclcpp::Publisher::wait_for_all_acked`). This is a fundamental limitation of ROS 2 (at the RMW API), and therefore ICEY cannot provide an awaitable `publish' function.

It would be required to add an asynchronous `async_wait_for_all_acked` to the RMW API so that an  an awaitable `publish` function can be implemented.

# References 

- [1] `await writer.WriteAsync` [using channels in C#](https://learn.microsoft.com/en-us/dotnet/core/extensions/channels)
- [2] `send(...).await` using [Tokio channels in Rust](https://tokio.rs/tokio/tutorial/channels)
- [3] [MQTT in C#](https://github.com/dotnet/MQTTnet/blob/980a5d0a6d58d77318056cd50d35602c34622360/Samples/Client/Client_Publish_Samples.cs#L39)