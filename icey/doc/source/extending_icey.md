# Extending ICEY

This tutorial shows how you can extend ICEY by adding a new subscriber.
In the following, we will extend ICEY to support the subscriber from the `image_transport` package which allows for subscribing to compressed camera images. Note that this tutorial explains what is already implemented in the `icey_image_transport.hpp` header.

You can always go on and read the API documentation on `Promise`, `Stream`, `impl::Stream` and `Context` for details.

## Declaring the custom Stream 

We essentially need to wrap the subscriber in a custom Stream. Then, each time the subscriber callback is called, we put the ROS message into this custom Stream.

We create a new `ImageTransportSubscriber` class that inherits from `icey::Stream`.
The class template `icey::Stream<Value, Error, BaseImpl>`
expects a `Value`, which is the ROS message type `sensor_msgs::Image::ConstSharedPtr `. As an `Error`, we will use `image_transport::TransportLoadException`, more on error handling later. 

The `BaseImpl` parameter needs some background information: 

An `icey::Stream` does not have any fields except a weak pointer to an `icey::impl::Stream<Base>` (PIMPL idiom). This allows them to be passed around by value (= clean code) while they actually refer to an unique object like a ROS subscriber. 

The class `impl::Stream<Base>` inherits from the given type `Base`, allowing it to be extended.

The `BaseImpl`-class holds everything a custom Stream needs as fields. A class deriving from `Stream` should never contain any fields because they may go out of scope. 

This is why we first need to create a custom impl `ImageTransportSubscriberImpl``that stores the subscriber:
```cpp
struct ImageTransportSubscriberImpl {
  image_transport::Subscriber subscriber;
};
```

Then we declare the actual custom stream `ImageTransportSubscriber` -- it gets the `ImageTransportSubscriberImpl` as the `BaseImpl`:

```cpp
struct ImageTransportSubscriber
    : public Stream<sensor_msgs::msg::Image::ConstSharedPtr,
                    image_transport::TransportLoadException, ImageTransportSubscriberImpl>
```

We can now write the constructor that takes the arguments we need for the subscriber -- topic name, QoS etc.
In addition, each stream constructor takes as its first argument the `icey::Context` corresponding to the ROS node.

```cpp
struct ImageTransportSubscriber
    : public Stream<sensor_msgs::msg::Image::ConstSharedPtr,
                    image_transport::TransportLoadException, ImageTransportSubscriberImpl> {

  using Base = Stream<sensor_msgs::msg::Image::ConstSharedPtr,
                      image_transport::TransportLoadException, ImageTransportSubscriberImpl>;
  ImageTransportSubscriber(Context &context, const std::string &base_topic_name,
                           const std::string &transport, const rclcpp::QoS qos,
                           const rclcpp::SubscriptionOptions &options = {})
      : Base(context) {
```

It is important to call the base class constructor and pass it the context, i.e. `Base(context)`. 

## Creating the subscriber

We create the subscriber inside the constructor of our custom Stream. To create the subscriber, we need the ROS node, we can obtain it through the `icey::Context`. For this you need to call `context.node_base()`, it gives you an `icey::NodeBase` object that abstracts regular nodes and lifecycle nodes. 
To get the underlying `rclcpp::Node`, you call `as_node()` on the `icey::NodeBase` object.

Since the `image_transport::create_subscription` needs a `rclcpp::Node *`, the constructor code becomes:

```cpp
 ImageTransportSubscriber(Context &context, const std::string &base_topic_name,
                           const std::string &transport, const rclcpp::QoS qos,
                           const rclcpp::SubscriptionOptions &options = {})
      : Base(context) {

    this->impl()->subscriber =
        image_transport::create_subscription(&context.node_base().as_node(), base_topic_name, callback,
                                            transport, qos.get_rmw_qos_profile(), options);
    
  }
```
This code is missing a declaration for `callback`, we will add it later.
The subscriber is stored with `this->impl()->subscriber` in the impl-object `ImageTransportSubscriberImpl` that we declared previously. You should also store everything inside the impl and never directly in the custom Stream.


```{note}
It is important to prefix member functions with `this->` to tell the compiler to look in the base class, otherwise you will get a compile error.
```


## Putting values in the stream

Now we implement the subscriber callback. It captures the Stream-impl and calls `put_value` for each new message:

```cpp
 ImageTransportSubscriber(Context &context, const std::string &base_topic_name,
                           const std::string &transport, const rclcpp::QoS qos,
                           const rclcpp::SubscriptionOptions &options = {})
      : Base(context) {
    
    /// The callback captures a weak reference to the Stream impl and puts the message:
    const auto callback = [impl = this->impl()](sensor_msgs::msg::Image::ConstSharedPtr image) {
      impl->put_value(image);
    };

    this->impl()->subscriber =
        image_transport::create_subscription(&context.node_base().as_node(), base_topic_name, callback,
                                            transport, qos.get_rmw_qos_profile(), options);
    
  }
```

```{warning}
You should never capture `this` in a lambda because the (outer) Stream-object has no guarantees about it's lifetime, only the impls are stored in the icey::Context.
Instead, you must only capture the impl with `impl = this->impl()`. 
````

And that's it already ! We have created a custom subscriber stream. To use it, we use the function `icey::Context::create_stream<T>` with our custom stream as `T`: 

```cpp
auto image_transport_sub = node->icey().create_stream<ImageTransportSubscriber>(topic_name, transport, qos);
```

## Error handling 

Streams support error handling -- they can hold either a value or an error. 
We can use this mechanism to handle the error that a compression algorithm was requested but not found. We modify the constructor to use a `try-catch` and put and error in case of an exception: 

```cpp
  ImageTransportSubscriber(Context &context, const std::string &base_topic_name,
                           const std::string &transport, const rclcpp::QoS qos,
                           const rclcpp::SubscriptionOptions &options = {})
      : Base(context) {
    assert_is_not_lifecycle_node(
        context);  /// NodeBookkeeping acts a type-erasing common interface between regular Nodes and
                /// lifecycle nodes, so we can only assert this at runtime
    const auto cb = [impl = this->impl()](sensor_msgs::msg::Image::ConstSharedPtr image) {
      impl->put_value(image);
    };
    try {
      this->impl()->subscriber =
          image_transport::create_subscription(&context.node_base().as_node(), base_topic_name, cb,
                                               transport, qos.get_rmw_qos_profile(), options);
    } catch (const image_transport::TransportLoadException &exception) {
      this->impl()->put_error(exception);
    }
  }
```