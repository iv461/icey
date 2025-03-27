# Extending ICEY

This tutorial shows how you can extend ICEY by adding a new subscription.
In the following, we will add the subscription from the `image_transport` package which allows for subscribing to compressed camera images. Note that everything this tutorial explains is already implemented in the `icey_image_transport.hpp` header.

You can always go on and read the API documentation on `Promise`, `Stream`, `impl::Stream` and `Context` for details.

## Declaring a custom Stream 

We essentially need to wrap the subscription in a custom Stream. Then, each time the subscription callback is called, we put the ROS message into this Stream.

We create a new `ImageTransportSubscription` class that inherits from `icey::Stream`.
The class template `icey::Stream<Value, Error, BaseImpl>`
expects a `Value`, which is the ROS message type `sensor_msgs::Image::ConstSharedPtr `. As an `Error`, we will use `image_transport::TransportLoadException`, more on error handling later. 

The `BaseImpl` parameter needs some background information: 

An `icey::Stream` does not have any fields except a weak pointer to an `icey::impl::Stream<Base>` (PIMPL idiom). This allows them to be passed around by value (= clean code) while they actually refer to an unique object like a ROS subscription. 

The class `impl::Stream<Base>` inherits from the given type `Base`, allowing it to be extended.

The `BaseImpl`-class holds everything a custom Stream needs as fields. A class deriving from `Stream` should never contain any fields because they may go out of scope. 

This is why we first need to create a custom impl `ImageTransportSubscriptionImpl` that stores the subscription:
```cpp
struct ImageTransportSubscriptionImpl {
  image_transport::Subscriber subscription;
};
```

Then we declare the actual custom stream `ImageTransportSubscription` -- it gets the `ImageTransportSubscriptionImpl` as the `BaseImpl`:

```cpp
struct ImageTransportSubscription
    : public Stream<sensor_msgs::msg::Image::ConstSharedPtr,
                    image_transport::TransportLoadException, ImageTransportSubscriptionImpl>
```

We can now write the constructor that takes the arguments we need for the subscription -- topic name, QoS etc.
In addition, each stream constructor takes as its first argument the `icey::Context` corresponding to the ROS node.

```cpp
struct ImageTransportSubscription
    : public Stream<sensor_msgs::msg::Image::ConstSharedPtr,
                    image_transport::TransportLoadException, ImageTransportSubscriptionImpl> {

  using Base = Stream<sensor_msgs::msg::Image::ConstSharedPtr,
                      image_transport::TransportLoadException, ImageTransportSubscriptionImpl>;
  ImageTransportSubscription(Context &context, const std::string &base_topic_name,
                           const std::string &transport, const rclcpp::QoS qos,
                           const rclcpp::SubscriptionOptions &options = {})
      : Base(context) {
```

It is important to call the base class constructor and pass it the context, i.e. `Base(context)`. 

## Creating the subscription

We create the subscription inside the constructor of our custom Stream. To create the subscription, we need the ROS node, we can obtain it through the `icey::Context`. For this you need to call `context.node_base()`, it gives you an `icey::NodeBase` object that abstracts regular nodes and lifecycle nodes. 
To get the underlying `rclcpp::Node`, you call `as_node()` on the `icey::NodeBase` object.

Since the `image_transport::create_subscription` needs a `rclcpp::Node *`, we create the subscription with:

```cpp
 ImageTransportSubscription(Context &context, const std::string &base_topic_name,
                           const std::string &transport, const rclcpp::QoS qos,
                           const rclcpp::SubscriptionOptions &options = {})
      : Base(context) {

    this->impl()->subscription =
        image_transport::create_subscription(&context.node_base().as_node(), base_topic_name, callback,
                                            transport, qos.get_rmw_qos_profile(), options);
    
  }
```
This code is missing a declaration for `callback`, we will add it in a moment.
The subscription is stored with `this->impl()->subscription` in the impl-object `ImageTransportSubscriptionImpl` that we declared previously. You should never store something directly in the Stream but only in the impl.
By storing it in the impl, we also achieve that the lifetime of the subscription is bound to the lifetime of the node: The `icey::Context` creates and owns all impls.


```{note}
It is important to prefix member functions with `this->` to tell the compiler to look in the base class, otherwise you will get a compile error.
```


## Putting values in the stream

Now we implement the subscription callback. It captures the Stream-impl and calls `put_value` for each new message:

```cpp
 ImageTransportSubscription(Context &context, const std::string &base_topic_name,
                           const std::string &transport, const rclcpp::QoS qos,
                           const rclcpp::SubscriptionOptions &options = {})
      : Base(context) {
    
    /// The callback captures a weak reference to the Stream impl and puts the message:
    const auto callback = [impl = this->impl()](sensor_msgs::msg::Image::ConstSharedPtr image) {
      impl->put_value(image);
    };

    this->impl()->subscription =
        image_transport::create_subscription(&context.node_base().as_node(), base_topic_name, callback,
                                            transport, qos.get_rmw_qos_profile(), options);
    
  }
```

```{warning}
You should never capture `this` in a lambda because the (outer) Stream-object has no guarantees about it's lifetime, only the impls are stored in the `icey::Context`.
Instead, you must only capture a weak pointer to the impl with `impl = this->impl()`. 
```

## Error handling 

Streams support error handling -- they can hold either a value or an error. 
We can use this mechanism to handle the error that a compression algorithm was requested but not found. We modify the constructor to use a `try-catch` and put an error in case of an exception: 

```cpp
    try {
      this->impl()->subscription =
          image_transport::create_subscription(&context.node_base().as_node(), base_topic_name, cb,
                                               transport, qos.get_rmw_qos_profile(), options);
    } catch (const image_transport::TransportLoadException &exception) {
      this->impl()->put_error(exception);
    }
```

## Using the new Stream 

We now have created a custom subscription stream `ImageTransportSubscription`. To use it, we call `icey::Context::create_stream<T>` with our custom stream as `T` and pass it all arguments that the constructor expects (except the `icey::Context`): 

```cpp
ImageTransportSubscription image_transport_sub = node->icey().create_stream<ImageTransportSubscription>(topic_name, transport, qos);
```

See also the [image transport example](../../icey_examples/src/using_image_transport.cpp)