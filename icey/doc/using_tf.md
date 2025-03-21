

# Promise mode 

In the following, we explain based on example patterns why this is what you actually want most of the time.
After this motivation, we explain how TF works in ICEY.


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