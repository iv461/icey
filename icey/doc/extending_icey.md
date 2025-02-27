# Extending ICEY

This tutorial shows how you can extend ICEY -- add new publishers, subscribers, 
but also new filters. 

## The Stream concept
For this, we first need to understand the concept `Stream` more in detail. 
An `Stream` is a value with a list of callbacks that get called when this value changes.
We can set the value of the Stream, this triggers that all the callbacks get called. 
We can also listen on changes to this value by registering a callback ourselves.

## Extending with custom subscribers and publishers: ìmage_transport

In the following, we will extend ICEY to support the subscribers from the `ìmage_transport` package that allows subscribing to compressed camera images.

We already know how to create new subscribers: When we subscribe to a topic in ROS, we pass it a subscriber-callback that gets called every time a new message is received. We will create therefore an Stream that holds as value the ROS-message. We will set the value of the Stream to the new ROS-message each time the subscriber-callback gets called. 

We create a new `ImageTransportSubscriber`-stream by deriving from the class `Stream`.
The class template `Stream<Value, Error, DerivedImpl>`
is parametrized on the `Value` it holds, for this we use the ROS-message type `sensor_msgs::Image::ConstSharedPtr `.

TODO finish:


We first declare the `DerivedImpl`-class that holds everything this stream needs as data fields. The class that actually derives from `Stream` should never contain any fields. 

```cpp
struct ImageTransportSubscriberImpl  {
  /// The image_transport subs/pubs use PIMPL, so we can hold them by value.
  image_transport::Subscriber subscriber;
};

class ImageTransportSubscriber : Stream< sensor_msgs::msg::Image::ConstSharedPtr,
                    image_transport::TransportLoadException, ImageTransportSubscriberImpl> {
public:

```
We can now write the constructor that takes the arguments we need for the subscriber -- topic name, QoS:


```cpp
class ImageTransportSubscriber : icey::Stream< sensor_msgs::Image::ConstSharedPtr > {
public:
    ImageTransportSubscriber(std::string topic_name, std::string transport, rclcpp::QoS qos) {

    }
    iamge_transport::Subscriber subscriber_;
};
```
we also created the field `subscriber_` to store the subscriber we will create.

### Attaching to the ROS-Node
Now comes the important part: The *attaching* to the ROS-Node. In ICEY, attaching means actually creating the subscriber/publishers/timers etc in the ROS-node. We do not do this immediatelly after constructing the Streams, instead we defer this process. It happens when the  method `attach` is called. 

We implement the method attach with a lambda-function, capturing by value (copying inside the lambda) all the parameters required for the subscriber: 


```cpp
class ImageTransportSubscriber : icey::Stream< sensor_msgs::Image::ConstSharedPtr > {
public:
    ImageTransportSubscriber(const std::string &topic_name, std::string &transport,  rclcpp::QoS qos) {  
        this->impl()->attach_ = [impl = this->impl(), topic_name, transport, qos](icey::NodeBookkeeping &node) {
            this->impl()->subscriber = image_transport::create_subscriber(node, transport, qos, subscriber_callback);
        };
    }
};
```

> Note: `this->` is important in the following to tell the compiler to look in the base class, otherwise it cannot find the function

We now created the subscriber as we normally would in ROS. 

### Setting the value of the Stream

We now make the Stream do something by implementing the subscriber callback to set the value.
The stream stores a pointer to the actual implementation, the `impl::Stream` that we first get with `impl()` and then call `put_value` inside the callback on it with the message:
```cpp
class ImageTransportSubscriber : icey::Stream< sensor_msgs::Image::ConstSharedPtr > {
public:
    ImageTransportSubscriber(const std::string &topic_name, std::string &transport,  rclcpp::QoS qos) {  
        auto stream_impl = this->impl(); /// Get the stream implementation
        /// Now implement the callabck to put_value with the message:
        auto subscriber_callback = [stream_impl](sensor_msgs::Image::ConstSharedPtr message) {
            stream_impl->put_value(message);
        };

        this->attach_ = [this, topic_name, transport, qos](icey::NodeBookkeeping &node) {
            subscriber_ = image_transport::create_subscriber(node, transport, qos, 
                    subscriber_callback);
        };
    }
    iamge_transport::Subscriber subscriber_;
};
```

And that's it already ! We have created a custom subscriber stream. To use it, we use the function `create_stream<T>` with our custom stream: 

```cpp
auto image_transport_sub = icey::create_stream<ImageTransportSubscriber>(topic_name, transport, qos);
```
or using the class-based API: 

```cpp
auto image_transport_sub = icey().create_stream<ImageTransportSubscriber>(topic_name, transport, qos);
```

### Writing a publisher 

Creating a custom publisher is very similar to the subscriber, but this time we react on changes and publish to ROS. We again use `image_transport` as an example: 

```cpp
class ImageTransportPublisher : icey::Stream< sensor_msgs::Image::SharedPtr > {
public:
    ImageTransportPublisher(const std::string &topic_name, rclcpp::QoS qos) {  
        this->attach_ = [impl = this->impl(), topic_name, qos](icey::NodeBookkeeping &node) {
            /// Create the publisher
            auto publisher = image_transport::create_publisher(node, qos);
        };
    }
};
```

This time however, we listen to changes of the Stream by calling `register_handler` and publish the message: 

```cpp
class ImageTransportPublisher : icey::Stream< sensor_msgs::Image::SharedPtr > {
public:
    ImageTransportPublisher(const std::string &topic_name, rclcpp::QoS qos) {  
        this->attach_ = [impl = this->impl(), topic_name, qos](icey::NodeBookkeeping &node) {
            /// Create the publisher
            auto publisher = image_transport::create_publisher(node, qos);
            impl->register_handler([publisher](const auto &new_state) {
                publisher.publish(new_state.value());
            })
        };
    }
};
```

### Error handling 

TODO 
Streams also support error handling, they can hold an `ErrorValue`:

