## Using TF 

Using tf with ICEY is a bit different that it is usually done. 
Usually, we see patterns like these: 

1: Getting the transform at the time the measurement was done (Source: [Nav2 Stack](https://github.com/ros-navigation/navigation2/blob/main///nav2_amcl/src/amcl_node.cpp#L577)): 
```cpp
/// On receiving a measurement, for example an image:
void on_camera_iamge(sensor_msgs::Image::SharedPtr img) {
    auto camera_stamp = img->header.stamp;
    /// Get the pose of the camera with respect to the map at the time the image was shot:
    auto cam_to_map = tf_buffer_->lookupTransform(cam_frame_, map_frame_, tf2_ros::fromMsg(camera_stamp));
}
```

So, we are usually always interested in getting the transform for a time of another topic: We are therefore **synchronizing** the transform with another topic.


Other variants of this pattern, that are rather anti-patterns are:

1. Assuming the latest transform in the buffer is approximately the same as the stamp (Source: [Nav2 Stack](https://github.com/ros-navigation/navigation2/blob/main//nav2_costmap_2d/plugins/costmap_filters/keepout_filter.cpp#L177) ):
```cpp

void on_camera_iamge(sensor_msgs::Image::SharedPtr img) {
    /// Get the latest transform in the buffer 
    auto cam_to_map = tf_buffer_->lookupTransform(cam_frame_, map_frame_, tf2::TimePointZero);
}
```


2. Taking as the timestamp the time in the callback, meaning the time the message was received instead of the time the measurement (= image) was taken:
```cpp

void on_camera_iamge(sensor_msgs::Image::SharedPtr img) {
    /// Get the latest transform in the buffer 
    auto cam_to_map = tf_buffer_->lookupTransform(cam_frame_, map_frame_, tf2::TimePointZero);
}
```

These patterns are found everywhere in the Nav2 stack for example when searching for `lookupTransform`.

## Solution in ICEY

In ICEY, you subscribe to a transform between two frames and use this as a signal:

```cpp
auto cam_to_map_signal = icey::create_transform("camera_frame", "map");
cam_to_map.then([](const geometry_msgs::TransfromStamped&cam_to_map) {
    /// Process updated transform
});
```

But how do we interpolate the transforms in the buffer to get the transform for a given time ? 

This is where we use the `SynchExactTime` filter, which always outputs something since the transform signal can be interpolated: 

```cpp
auto camera_feed = icey::create_signal("camera_front");
auto cam_to_map_signal = icey::create_transform("camera_frame", "map");

icey::SyncExactTime(camera_feed, cam_to_map_signal).then([](sensor_msgs::Camera::ConstSharedPtr image, geometry_msgs::TransfromStamped::ConstSharedPtr camera_to_map) {

});

```

The exact-time synchronizer matches the header time stamps of two topics exactly. This filter usually won't output anything for topics that are not stamped exactly the same. But in this case, one of the signals, namely the transform, is `Interpolatable`: this means, the filter receives the image, and the looks up the transform for the header timestamp of the image. As usual, the filter outputs something only if the result can be syncrhonized, in this case this means if the transform is available and can be looked up. 

Overall, this means the above code is equivalent to this pattern:

```cpp

void on_image(sensor_msgs::Camera::ConstSharedPtr image) {
    geometry_msgs::TransfromStamped camera_to_map_transform;
    try {
        camera_to_map_transform = 
            tf_buffer_->lookupTransform("camera_frame", "map", tf2_ros::fromMsg(image->header.stamp));
    } catch(...) {
        return; // Return if the transform is not available yet.
    }

    /// Otherwise, process the synchronized image and camera_to_map_transform ...
}
```

You may have noticed one limitation: We cannot use the frame name that is inside the header of the image message, i.e. `header.frame_id`. This limitation is required however to identify a clear data-flow, a core concept of ICEY: If we would have to look up different frames depending on the `frame_id` inside the message, this would mean that 
we need to synchronize different sources (the different nodes that usually publish on the topic /tf) depending on the message **content**. This means, the data-flow would have to change dynamically, depending on the mssage-content. Generally, the data-flow graph is static in ICEY, so this is not possible. But it may be possible in the future if we realize this flexibility is really necessary.

Also, in practice the names of rigid coordinate systems (like sensor mounts) are fixed and known in priori, and do not change. The map coordinate system is named by convention [1] always `map`. 

Therefore, this limitation of having to know the frame names on subscription is not concerning in practice.

## Using the TF-buffer directly

You can also use the TF-buffer directly inside callbacks since it offers useful transform-functions: 

```cpp
auto camera_feed = icey::create_signal("camera_front");
auto cam_to_map_signal = icey::create_transform("camera_frame", "map");

icey::SyncExactTime(camera_feed, cam_to_map_signal).then([](sensor_msgs::Camera::ConstSharedPtr image, geometry_msgs::TransfromStamped::ConstSharedPtr camera_to_map) {

    icey::node->tf_buffer_->transform(...);
});

```

WARNING: Do not use the buffer however to call `lookupTransform` or `waitforTransform` under any circumstances. This will certainly lead to dead-locks and is also discouraged: You need to find a way to describe the data-flow statically, instead on relying on such asynchronous functions. 

# References: 

- [1] REP 105: Coordinate System naming conventions, available online: https://www.ros.org/reps/rep-0105.html