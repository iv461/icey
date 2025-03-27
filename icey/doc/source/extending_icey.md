# Extending ICEY

This tutorial shows how you can extend ICEY by adding a new subscriber.

You can always go on and read the API documentation on `Promise`, `Stream`, `impl::Stream` and `Context` for details.

## Extending with custom subscribers and publishers: image_transport

In the following, we will extend ICEY to support the subscribers from the `image_transport` package that allows subscribing to compressed camera images.

For this, we essentially need to wrap the subscriber in a new Stream. This Stream holds as value the ROS-message. We then put the ROS-message in the Stream each time the subscriber callback gets called. 

We create a new `ImageTransportSubscriber`-stream by deriving from the class `icey::Stream`.
The class template `Stream<Value, Error, BaseImpl>`
is parametrized on the `Value` it holds, for this we use the ROS-message type `sensor_msgs::Image::ConstSharedPtr `.

An `icey::Stream` does not to not have any fields except a weak reference to an `icey::impl::Stream` (PIMPL idiom). This allows them to be passed around by value (= clean code) while they actually referring to an unique object like a ROS subscriber. 

`impl::Stream<Base>` is a class that derives from the class `Base` that is given as a template parameter. 
This allows to extend the impl. 

The `BaseImpl`-class holds everything a custom Stream needs as fields. A class deriving from `Stream` should never contain any fields because they may go out of scope. 

```cpp
struct ImageTransportSubscriberImpl {
  /// The image_transport subs/pubs use PIMPL, so we can hold them by value.
  image_transport::Subscriber subscriber;
};

struct ImageTransportSubscriber
    : public Stream<sensor_msgs::msg::Image::ConstSharedPtr,
                    image_transport::TransportLoadException, ImageTransportSubscriberImpl>
```

We can now write the constructor that takes the arguments we need for the subscriber -- topic name, QoS etc.
Additionally, every Stream constructor takes as a first argument the `icey::Context` that corresponds to the ROS node.

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

### Creating the subscriber

In the constructor, we need to create the subscriber. We create ROS entities using the Context, for this you need to call `context.node()` to obtain an `icey::NodeBase` object that abstracts regular Nodes and lifecycle nodes. 
You can also obtain the underlying `rclcpp::Node` by calling `as_node()` on the `icey::NodeBase` object.

Since the `image_transport::create_subscription` needs a `rclcpp::Node *` object as, the code becomes:

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

> Note: `this->` is important in the following to tell the compiler to look in the base class, otherwise it cannot find the function

To store fields in the Stream, you should always use `this->impl()-><field>`.

### Putting values in the stream

We now make the Stream do something by implementing the subscriber callback.
We create a callback that captures the Stream-impl and puts the message into the stream:

```cpp
 ImageTransportSubscriber(Context &context, const std::string &base_topic_name,
                           const std::string &transport, const rclcpp::QoS qos,
                           const rclcpp::SubscriptionOptions &options = {})
      : Base(context) {
    
    /// The callback captures a weak reference to the Stream impl and puts the message:
    const auto cb = [impl = this->impl()](sensor_msgs::msg::Image::ConstSharedPtr image) {
      impl->put_value(image);
    };

    this->impl()->subscriber =
        image_transport::create_subscription(&context.node_base().as_node(), base_topic_name, callback,
                                            transport, qos.get_rmw_qos_profile(), options);
    
  }
```

```{warning}
You should never capture `this` in a lambda because the (outer) Stream-object has no guarantees about it's lifetime, only the impls are stored in the icey::Context.
You must instead always capture only the impl with `impl = this->impl()`. 
````

And that's it already ! We have created a custom subscriber stream. To use it, we use the function `icey::Context::create_stream<T>` with our custom stream as `T`: 

```cpp
auto image_transport_sub = node->icey().create_stream<ImageTransportSubscriber>(topic_name, transport, qos);
```

### Error handling 

TODO 
Streams also support error handling, they can hold an `ErrorValue`:

