# Publish and subscribe

In the following, we will look at how to create publishers and subscriptions.

## The Stream concept 

Streams are a fundamental concept upon which ICEY is built. An `icey::Stream` is an asynchronous abstraction that represents a sequence of values, potentially infinitely many. 
Subscriptions, service servers, and timers all model the stream concept. 
You can either register a callback on a stream that's invoked when a new value (i.e. ROS message) arrives, or you can asynchronously wait for a new value using `co_await stream`. 

Streams shine when we apply transformations to them -- synchronization, filtering, buffering, we can also just `publish()` a stream. More on this later.


## Subscribe

Subscriptions are created using `node->icey().create_subscription<Message>(<topic-name>, <qos>, <callback>={}, <options>={})`. The meaning of the arguments is the same as that to the `create_subscription`-function of a regular ROS-node. 

Example: 

```cpp
auto node = std::make_shared<icey::Node>("yolo_node");
icey::SubscriptionStream<sensor_msgs::msg::Image> camera_image = node->icey().create_subscription<sensor_msgs::msg::Image>("camera", 
  [](sensor_msgs::msg::Image::SharedPtr msg) {
      /// Consume camera message here ..
  },
  rclcpp::SensorDataQoS());
```

### Key differences to regular ROS are: 
  - `create_subscription` returns a `Stream` (you can access the `rclcpp::Subscription` using `stream.subscription`)
  - Asynchronous functions, i.e. coroutines, can be used as callbacks
  - The lifetime of the subscription is bound to the lifetime of the node: This means, you don't need to store  the subscription as a member in the node class (i.e. do bookkeeping)
  - The quality of service is optional: If it is not given, the so called *system default* one is used
  - The callback is optional: You do not need to provide a callback if you plan for example to first synchronize the subscription with another stream

## Publishers

Publishers are created using `node->icey().create_publisher<Message>(<topic-name>, <qos>, <options>={})`. The meaning of the arguments is the same as that to the `create_publisher`-function of a regular ROS-node. 

Example: 

```cpp
auto node = std::make_shared<icey::Node>("yolo_node");
icey::PublisherStream<sensor_msgs::msg::Image> publisher = node->icey().create_publisher<sensor_msgs::msg::Image>("camera", rclcpp::SensorDataQoS());

publisher.publish(message);
```

### Key differences to regular ROS are: 
  - The lifetime of the publisher is bound to the lifetime of the node: This means, you don’t need to store the publisher as a member in the node class (i.e. do bookkeeping)
  - `create_publisher` returns a `Stream`, (you can access the `rclcpp::Publisher` using `stream.publisher`)
  - If no quality of service is given, a uses the so called *system default*  

### Awaiting `publish`

Currently, due to a ROS limitation, it is not possible to await a call to publish.

Explanation:
Publishing a message with reliable quality of service involves two steps: (1) sending the message, and (2) waiting for an acknowledgment (ACK) from the receiver. Since we don't know when (or if) we will receive the ACK from another node, waiting for the ACK is an inherently asynchronous operation.
Therefore, calling publish is overall an asynchronous operation when using reliable quality of service.

In ROS however, publishing a message is treated as a fire-and-forget task. The message is sent, and no waiting is done for the ACK.
This differs from how other message-passing libraries implement a publish function [1, 2, 3, 4].

ROS provides an API for waiting on the ACK but only a synchronous one, `rclcpp::PublisherBase::wait_for_all_acked`. There is no asynchronous API (this is a limitation at the RMW-API layer). For this reason, ICEY cannot provide an awaitable publish function.

## References 

- [1] `await writer.WriteAsync` [using channels in C#](https://learn.microsoft.com/en-us/dotnet/core/extensions/channels)
- [2] `send(...).await` using [Tokio channels in Rust](https://tokio.rs/tokio/tutorial/channels)
- [3] [MQTT in C#](https://github.com/dotnet/MQTTnet/blob/980a5d0a6d58d77318056cd50d35602c34622360/Samples/Client/Client_Publish_Samples.cs#L39)
- [4] `await asyncPublish` using [MQTT.js in JavaScript](https://github.com/mqttjs/MQTT.js?tab=readme-ov-file#publish-async)