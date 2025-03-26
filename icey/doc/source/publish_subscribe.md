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

Key differences are: 
  - Asynchronous functions, i.e. coroutines, can be used as callbacks
  - The lifetime of the subscription is bound to the lifetime of the node: This means, you don't need to store  the subscriber as a member in the node class (i.e. do bookkeeping)
  - `create_subscriber` returns a `stream`, but you can access the `rclcpp::Subscriber` using `stream.subscriber` 
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

Key differences are: 
  - The lifetime of the publisher is bound to the lifetime of the node: You do not have to store the subscriber in the node class, i.e. do bookkeeping, ICEY does this for you
  - `create_publisher` returns a stream, but you can access the ROS-publisher using `steam.publisher` 
  - If no quality of service is given, a uses the so called *system default*  

publisher()
