# Using TF 

Using TF with ICEY is a bit different that in regular ROS: Since everything is asynchronous in ICEY, so are transforms. ICEY allows you to *subscribe* to a single transform between to coordinate systems and then *synchronize* it to another topic.

In the following, we explain based on example patterns why this is what you actually want most of the time.
After this motivation, we explain how TF works in ICEY.

The following patterns of using TF are the most often:

1: Getting the transform at the time the measurement was done (Source: [Nav2 Stack](https://github.com/ros-navigation/navigation2/blob/main///nav2_amcl/src/amcl_node.cpp#L577)): 
```cpp
/// On receiving a measurement, for example an image:
void on_camera_image(sensor_msgs::Image::SharedPtr img) {
    auto camera_stamp = img->header.stamp;
    /// Get the pose of the camera with respect to the map at the time the image was shot:
    auto cam_to_map = tf_buffer_->lookupTransform(cam_frame_, map_frame_, tf2_ros::fromMsg(camera_stamp));
}
```


Other variants of this pattern (that are rather anti-patterns) are:

1. Just taking the last transform in the buffer to avoid waiting, and therefore assuming the latest transform in the buffer is approximately the same as the message stamp (Source: [Nav2 Stack](https://github.com/ros-navigation/navigation2/blob/main//nav2_costmap_2d/plugins/costmap_filters/keepout_filter.cpp#L177) ):
```cpp

void on_camera_iamge(sensor_msgs::Image::SharedPtr img) {
    /// Get the latest transform in the buffer 
    auto cam_to_map = tf_buffer_->lookupTransform(cam_frame_, map_frame_, tf2::TimePointZero);
}
```


2. Looking up at the current time in the callback: This is the time the message was received, not the time the measurement was made (which may have been multiple milliseconds earlier):
```cpp

void on_camera_iamge(sensor_msgs::Image::SharedPtr img) {
    ...
    auto cam_to_map = tf_buffer_->lookupTransform(cam_frame_, map_frame_, this->get_clock().now());
    ...
}
```


So, we are usually always interested in getting the transform for a time of another topic: We are therefore *synchronizing* the transform with another topic.

## How TF works in ICEY
In ICEY, you *subscribe* to a transform between two frames:

```cpp
auto map_base_link_tf = node->icey().create_transform_subscription("map", "base_link");

auto theta_angle = map_base_link_tf
    .then([&](const geometry_msgs::msg::TransformStamped &new_transform) {
        std_msgs::msg::Float32 out_msg;
        const double cos_theta_half = new_transform.transform.rotation.z;
        const double theta = std::acos(2. * cos_theta_half)
        RCLCPP_INFO_STREAM(node->get_logger(), "Received a new transform, orientation angle theta was: " << cos_theta_half);
        return theta;
    });
    
theta_angle.publish("theta_angle");
```
To handle the lookup at a given time, we simply call `icey.synchronize` with another subscription. This will call `lookupTransform` with the header stamp everyt time a new message arrives. Our previous example becomes in ICEY:

```cpp
auto camera_image = node->icey().create_subscription<sensor_msgs::msg::Image>("camera_front");
auto cam_to_map_transform = node->icey().create_transform_subscription("camera_frame", "map");

node->icey().synchronize(camera_image, cam_to_map_transform)
    .then([](sensor_msgs::Camera::SharedPtr image, geometry_msgs::TransfromStamped::SharedPtr camera_to_map) {

    });
```

The above ICEY-code is equivalent to this pattern:

```cpp

void on_image(sensor_msgs::Camera::SharedPtr image) {
    geometry_msgs::TransfromStamped camera_to_map_transform;
    try {
        camera_to_map_transform = 
            tf_buffer_->lookupTransform("camera_frame", "map", tf2_ros::fromMsg(image->header.stamp));
    } catch(...) {
        return; // Return if the transform is not available yet.
    }

}
```

You may have noticed one limitation: We cannot use the frame name that is inside the header of the image message, i.e. `header.frame_id`. This limitation is required however to identify a clear data-flow, a core concept of ICEY: If we would have to look up different frames depending on the `frame_id` inside the message, this would mean that 
we need to synchronize different sources (the different nodes that usually publish on the topic /tf) depending on the message **content**. This means, the data-flow would have to change dynamically, depending on the mssage-content. Generally, the data-flow graph is static in ICEY, so this is not possible. But it may be possible in the future if we realize this flexibility is really necessary.

Also, in practice the names of rigid coordinate systems (like sensor mounts) are fixed and known in priori, and do not change. The map coordinate system is named by convention [1] always `map`. 

Therefore, this limitation of having to know the frame names on subscription is not concerning in practice.

## Using the TF-buffer directly

You can also use the TF-buffer directly inside callbacks since it offers useful transform-functions: 

```cpp
auto camera_feed = icey::create_subscription("camera_front");
auto cam_to_map_transfom = icey::create_transform_subscription("camera_frame", "map");

icey::synchronize(camera_feed, cam_to_map_transfom).then([](sensor_msgs::Camera::ConstSharedPtr image, geometry_msgs::TransfromStamped::ConstSharedPtr camera_to_map_tf) {

    icey::node->tf_buffer_->transform(...);
});

```

```{warning}
Do not use the buffer to call `lookupTransform` or `waitforTransform` directly. This will certainly lead to dead-locks and is also discouraged: You need to find a way to describe the data-flow statically, instead on relying on such asynchronous functions. 
```

```{note}
Subscribing to high-frequency TF topics is a known issue in ROS that can cause performance problem, solutions include [2,3,4].
```

## References: 

- [1] REP 105: Coordinate System naming conventions, available online: https://www.ros.org/reps/rep-0105.html
- [2] https://github.com/magazino/tf_service
- [3] https://github.com/peci1/tf2_server
- [4] https://github.com/bit-bots/bitbots_tf_buffer