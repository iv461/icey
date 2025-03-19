# Using TF 

Coordinate systems are communicated in ROS over the topic `/tf` and `/tf_static`, so receiving them is an inherently asynchronous operation.
To obtain coordinate system transforms, ICEY provides again an async/await based API. 

ICEY also allows you to *subscribe* to a single transform between to coordinate system instead of requesting a transform at a specific time. This features is also useful in some applications. 

# Looking up transforms: 

To lookup a transform at a specific time, you first create in ICEY a `icey::TransformBuffer`: This is the usual combination of a subscriber and a buffer bundled in a single object. 

You can then call `lookup` and await it: 

```cpp 
icey::TransformBuffer tf_buffer = node->icey().create_transform_buffer();

node->icey()
      .create_subscription<sensor_msgs::msg::PointCloud2>("/icey/test_pcl")
      // Use a coroutine (an asynchronous function) as a callback for the subscriber:
      .then([&tf_buffer,
             &node](sensor_msgs::msg::PointCloud2::SharedPtr point_cloud) -> icey::Promise<void> {
    icey::Result<geometry_msgs::msg::TransformStamped, std::string> tf_result =
            co_await tf_buffer.lookup("map", point_cloud->header.frame_id,
                                      icey::rclcpp_to_chrono(point_cloud->header.stamp), 200ms);
        /// Continue transforming the point cloud here ..
    });
```

See also the [TF lookup](../../icey_examples/src/tf_lookup_async_await.cpp) example.

# Promise mode 

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

You may have noticed one limitation: We cannot use the frame name that is inside the header of the image message, i.e. `header.frame_id`. 
We can use for this another function `synchronize_with_transform(<target-frame>)` that will use the `header.frame_id` as the source frame: 

```cpp
auto camera_image = node->icey().create_subscription<sensor_msgs::msg::Image>("camera_front");
const std::string target_frame = "map";
/// This will use the header.frame_id of the Image as the source frame:

camera_image.synchronize_with_transform(target_frame)
    .then([](sensor_msgs::Camera::SharedPtr image, geometry_msgs::TransfromStamped::SharedPtr camera_to_map) {

    });
```

You can even use parameters as a target frame:

```cpp
auto camera_image = node->icey().create_subscription<sensor_msgs::msg::Image>("camera_front");
auto target_frame = node->icey().create_parameter<std::string>("target_frame");

/// This will use the header.frame_id of the Image as the source frame and the current parameter value as the target_frame (automatically receiving updates)
camera_image.synchronize_with_transform(target_frame)
    .then([](sensor_msgs::Camera::SharedPtr image, geometry_msgs::TransfromStamped::SharedPtr camera_to_map) {

    });
```

```cpp
auto camera_image = node->icey().create_subscription<sensor_msgs::msg::Image>("camera_front");
auto cam_to_map_transform = node->icey().create_transform_subscription("camera_frame", "map");

node->icey().synchronize(camera_image, cam_to_map_transform)
    .then([](sensor_msgs::Camera::SharedPtr image, geometry_msgs::TransfromStamped::SharedPtr camera_to_map) {

    });
```

In practice the names of rigid coordinate systems (like sensor mounts) are fixed and known in priori, and do not change. The map coordinate system is named by convention [1] always `map`. 

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


```{note}
Subscribing to high-frequency TF topics is a known issue in ROS that can cause performance problem, solutions include [2,3,4].
```

## References: 

- [1] REP 105: Coordinate System naming conventions, available online: https://www.ros.org/reps/rep-0105.html
- [2] https://github.com/magazino/tf_service
- [3] https://github.com/peci1/tf2_server
- [4] https://github.com/bit-bots/bitbots_tf_buffer