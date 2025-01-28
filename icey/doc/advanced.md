# Advanced topics 

### Order of initialization 

In ICEY, the parameters are initialized first, then the subscribers, then the publishers, then the services, then the actions and 
only then the timers. 

The order of declaration is important to make certain correctness properties. 
The parameters are initialized first because their values may be needed for initializing the other things. 


TODO clarify order 


# Enabling using `image_transport::Subscriber` using additional subscribe/publish filters 

There are multiple occasions in ROS where you have to use a different way to create publishers and subscribers, for example to subscribe to compressed sensor data. 

For example, for decompressing image data, you use `image_transport::Subscriber` like this: 

```cpp
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
class CompressedImageSubscriberNode : public rclcpp::Node {
public:
    CompressedImageSubscriberNode() : Node("compressed_image_subscriber")    {
        // Initialize the ImageTransport object with the node
        image_transport::ImageTransport it(this);
        // Subscribe to the compressed image topic
        image_subscriber_ = it.subscribe("image/compressed", 1, 
            std::bind(&CompressedImageSubscriberNode::imageCallback, this, std::placeholders::_1));
    }
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) { /// Process image .. 
    }
```

This concept is essentially also a filter, but unfortunatelly it is currently not really unified in ROS. 
There are different kind of subscribers, depending on what you want to achieve, e.g. `message_filters::Subscriber` or a filter that, after receiving a message, first decompresses it. This is common for visual sensors like cameras and LiDAR-sensors that produce a lot of data. 
Unfortunatelly, even for different visual sensors the `image_transport` concept is not unified and instead you have to use different subscriber. 


We can enable the same in ICEY using TODO
