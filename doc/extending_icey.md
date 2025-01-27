# Extending ICEY

This tutorial shows how you can extend ICEY -- add new publishers, subscribers, 
but also new filters. 
For this, we first need to understand the concept `Observable` more in detail. 
An `Observable` is a value with a list of callbacks that get called when this value changes.
We can set the value of the Observable, this triggers that all the callbacks get called. 
We can also listen on changes to this value by registering a callback ourselves.

With this information, we already know how to create new subscribers: When we subscribe to a topic in ROS, we pass it a subscriber-callback that gets called every time a new message is received. We will create therefore an Observable that holds as value the ROS-message. We will set the value of the Observable to the new ROS-message each time the subscriber-callback gets called. 

> ###  `image_transport` Subscriber
In the following, we will extend ICEY to support the subscribers from the `ìmage_transport` package that allows subscribing to compressed camera images.

We create a new `ImageTransportSubscriber`-observable by deriving from the class `Observable` and giving it as a template parameter the ROS-message type `sensor_msgs::Image::ConstSharedPtr `:

```cpp
class ImageTransportSubscriber : icey::Observable< sensor_msgs::Image::ConstSharedPtr > {
public:

```
We can now write the constructor that takes the arguments we need for the subscriber -- topic name, QoS:


```cpp
class ImageTransportSubscriber : icey::Observable< sensor_msgs::Image::ConstSharedPtr > {
public:
    ImageTransportSubscriber(std::string topic_name, std::string transport, rclcpp::QoS qos) {

    }
    iamge_transport::Subscriber subscriber_;
};
```
we also created the field `subscriber_` to store the subscriber we will create.

### Attaching to the ROS-Node
Now comes the important part: The *attaching* to the ROS-Node. In ICEY, attaching means actually creating the subscriber/publishers/timers etc in the ROS-node. We do not do this immediatelly after constructing the Observables, instead we defer this process. It happens when the  method `attach` is called. 

We implement the method attach with a lambda-function, capturing by value (copying inside the lambda) all the parameters required for the subscriber: 


```cpp
class ImageTransportSubscriber : icey::Observable< sensor_msgs::Image::ConstSharedPtr > {
public:
    ImageTransportSubscriber(const std::string &topic_name, std::string &transport,  rclcpp::QoS qos) {  
        this->attach_ = [this, topic_name, transport, qos](icey::NodeBookkeeping &node) {
            subscriber_ = image_transport::create_subscriber(node, transport, qos, 
                    subscriber_callback);
        };
    }
    iamge_transport::Subscriber subscriber_;
};
```

> Note: `this->` is important in the following to tell the compiler to look in the base class, otherwise it cannot find the function

We now created the subscriber as we normally would in ROS. 

### Setting the value of the Observable

We now make the Observable do something by implementing the subscriber callback to set the value.
The observable stores a promise that we first get with `get_promise` and then `resolve` it with the message inside the callback:
```cpp
class ImageTransportSubscriber : icey::Observable< sensor_msgs::Image::ConstSharedPtr > {
public:
    ImageTransportSubscriber(const std::string &topic_name, std::string &transport,  rclcpp::QoS qos) {  
        auto promise = this->get_promise(); /// Get the promise holding the value
        /// Now implement the callabck to resolve with the message:
        auto subscriber_callback = [promise](sensor_msgs::Image::ConstSharedPtr message) {
            promise->resolve(message);
        };

        this->attach_ = [this, topic_name, transport, qos](icey::NodeBookkeeping &node) {
            subscriber_ = image_transport::create_subscriber(node, transport, qos, 
                    subscriber_callback);
        };
    }
    iamge_transport::Subscriber subscriber_;
};
```

And that's it already ! We have created a custom subscriber observable. To use it, we use the function `create_observable<T>` with our custom observable: 

```cpp
auto image_transport_sub = icey::create_observable<ImageTransportSubscriber>(topic_name, transport, qos);
```
or using the class-based API: 

```cpp
auto image_transport_sub = icey().create_observable<ImageTransportSubscriber>(topic_name, transport, qos);
```

### Writing a publisher 

Creating a custom publisher is very similar to the subscriber, but this time we react on changes and publish to ROS. We again use `image_transport` as an example: 

```cpp
class ImageTransportPublisher : icey::Observable< sensor_msgs::Image::SharedPtr > {
public:
    ImageTransportPublisher(const std::string &topic_name, rclcpp::QoS qos) {  
        auto promise = this->get_promise(); /// Get the promise holding the value
        this->attach_ = [this, topic_name, qos](icey::NodeBookkeeping &node) {
            /// Create the publisher
            auto publisher = image_transport::create_publisher(node, qos);
        };
    }
};
```

This time however, we listen to changes of the Observable by calling `register_handler` on the promise and publish the message: 

```cpp
class ImageTransportPublisher : icey::Observable< sensor_msgs::Image::SharedPtr > {
public:
    ImageTransportPublisher(const std::string &topic_name, rclcpp::QoS qos) {  
        auto promise = this->get_promise(); /// Get the promise holding the value
        this->attach_ = [promise, topic_name, qos](icey::NodeBookkeeping &node) {
            /// Create the publisher
            auto publisher = image_transport::create_publisher(node, qos);

            promise->register_handler([promise, publisher]() {
                publisher.publish(promise.value());
            })
        };
    }
};
```

### Error handling 

Observables also support error handling, they can hold an `ErrorValue`:

