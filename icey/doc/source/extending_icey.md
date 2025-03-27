# Extending ICEY

This tutorial shows how you can extend ICEY by adding a new subscriber.

You can always go on and read the API documentation on `Promise`, `Stream`, `impl::Stream` and `Context` for details.

## Extending with custom subscribers and publishers: image_transport

In the following, we will extend ICEY to support the subscribers from the `image_transport` package which allow subscribing to compressed camera images.

For this, we essentially need to wrap the subscriber in a custom Stream. We then put the ROS message into this custom Stream each time the subscriber callback is called. 

We create a new `ImageTransportSubscriber` class that inherits from `icey::Stream`.
The class template `icey::Stream<Value, Error, BaseImpl>`
expects a `Value`, which is the ROS message type `sensor_msgs::Image::ConstSharedPtr `. As an `Error`, we will use `image_transport::TransportLoadException`, more on error handling later. 

The `BaseImpl` parameter needs some background information: 

An `icey::Stream` does not have any fields except a weak pointer to an `icey::impl::Stream<Base>` (PIMPL idiom). This allows them to be passed around by value (= clean code) while they actually referring to an unique object like a ROS subscriber. 

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

We create the subscriber inside the constructor of our custom Stream. For creating the subscriber, we need the ROS node, we can obtain it through the `icey::Context`. For this you need to call `context.node_base()`, it gives you a `icey::NodeBase` object that abstracts regular Nodes and lifecycle nodes. 
To obtain the underlying `rclcpp::Node`, you call `as_node()` on the `icey::NodeBase` object.

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
In this code, a declaration for `callback` is missing, we will add it in a second. 
The subscriber is stored with `this->impl()->subscriber` in the impl-object `ImageTransportSubscriberImpl` which we declared previously. You should also store everything inside the impl and never directly in the custom Stream.


```{note}
It is important to prefix member functions with `this->` to tell the compiler to look in the base class, otherwise you will get a compile error.
```


### Putting values in the stream

We now make the Stream do something by implementing the subscriber callback.
We create a callback that captures the Stream-impl and puts the message into the stream:

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
You must instead always capture only the impl with `impl = this->impl()`. 
````

And that's it already ! We have created a custom subscriber stream. To use it, we use the function `icey::Context::create_stream<T>` with our custom stream as `T`: 

```cpp
auto image_transport_sub = node->icey().create_stream<ImageTransportSubscriber>(topic_name, transport, qos);
```

### Error handling 

TODO 
Streams also support error handling, they can hold an `ErrorValue`:

