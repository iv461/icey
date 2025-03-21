# Using transforms (TF)

Coordinate system transforms are communicated in ROS over the topic `/tf` and `/tf_static`, so receiving them is inherently an asynchronous operation.
ICEY provides different ways for obtaining transforms: The usual `lookup` function (with async/await API), but also synchronizing a topic like a point cloud with a transform. ICEY even allows *subscribing* to a transform: A callback will be called every time a transform between two coordinate systems changes.

## Looking up transforms: 

To lookup a transform at a specific time, you first create a `icey::TransformBuffer`: This is the usual combination of a subscriber on `/tf`/`/tf_static` and a buffer bundled in a single object. 

You can then call `lookup` and await it: 

```cpp 
icey::TransformBuffer tf_buffer = node->icey().create_transform_buffer();

node->icey()
      // Use a coroutine (an asynchronous function) as a callback for the subscriber:
      .create_subscription<sensor_msgs::msg::PointCloud2>("/icey/test_pcl", 
        [&tf_buffer, &node](sensor_msgs::msg::PointCloud2::SharedPtr point_cloud) -> icey::Promise<void> {

        auto tf_result = co_await tf_buffer.lookup("map", point_cloud->header.frame_id,
                                      icey::rclcpp_to_chrono(point_cloud->header.stamp), 200ms);

        if (tf_result.has_value()) {
          geometry_msgs::msg::TransformStamped transform_to_map = tf_result.value();
          /// Continue transforming the point cloud here ..
        } else {
          RCLCPP_INFO_STREAM(node->get_logger(), "Transform lookup error " << tf_result.error());
        }

      });
```
See also the [TF lookup](../../icey_examples/src/tf_lookup_async_await.cpp) example.

The code that follows `co_await tf_buffer.lookup` will execute only after the transform is available (or a timeout occurs) -- the behavior is therefore the same to the regular `lookupTransform` function that you are used to, no surprises.

The difference is that in ICEY, the `lookup`-call is asynchronous while the original ROS API (`tf2_ros::Buffer::lookupTransform`) is synchronous, it does *"busy-wait"* with a separate executor.

ICEY on the other hand provides an API allows to write *synchronously-looking* code using async/await, consistent with  the APIs of other inherently asynchronous operations like service calls.

This difference is rather subtle and shouldn't be too relevant if you are only a user of TF. 
TODO maybe write mode

## Synchronizing and transforming data 

Another way of obtaining transforms is to *declare* that you need a transform for every point in time of another sensor measurement like a point cloud was made. This is on contrast to the *imperative* style
where you explicitly request every single transform at a given time via a call to `lookup`.

With the declarative style you can essentially say *"I need this point cloud transformed in the map frame before further processing"* 

### Motivation 
To see why this is useful, we will analyze some common usage patterns of `lookupTransform`. 

1: Getting the transform at the time the measurement was done:

```cpp
/// On receiving a measurement, for example an image:
void on_camera_image(sensor_msgs::Image::SharedPtr img) {
    auto camera_stamp = img->header.stamp;
    /// Get the pose of the camera with respect to the map at the time the image was shot:
    auto cam_to_map = tf_buffer_->lookupTransform(cam_frame_, map_frame_, tf2_ros::fromMsg(camera_stamp), 200ms);
}
```
Other variants of this pattern (that are rather anti-patterns) are:

1. Just taking the last transform in the buffer to avoid waiting, and therefore assuming the latest transform in the buffer is approximately the same as the message stamp (Source: [Nav2 Stack](https://github.com/ros-navigation/navigation2/blob/main//nav2_costmap_2d/plugins/costmap_filters/keepout_filter.cpp#L177) ):
```cpp

void on_camera_image(sensor_msgs::Image::SharedPtr img) {
    /// Get the latest transform in the buffer 
    auto cam_to_map = tf_buffer_->lookupTransform(cam_frame_, map_frame_, tf2::TimePointZero);
}
```

2. Looking up at the current time in the callback: This is the time the message was received, not the time the measurement was made (which may have been multiple milliseconds earlier):
```cpp

void on_camera_image(sensor_msgs::Image::SharedPtr img) {
    ...
    auto cam_to_map = tf_buffer_->lookupTransform(cam_frame_, map_frame_, this->get_clock().now());
    ...
}
```

So, we are usually always interested in getting the transform for a time of another topic: We are therefore *synchronizing* the transform with another topic.

### Synchronizing with transform with ICEY:

ICEY supports this kind synchronization with the transform: 

```cpp 
node->icey()
      .create_subscription<sensor_msgs::msg::PointCloud2>("/icey/test_pcl")
      .synchronize_with_transform("map", 200ms)
      .unwrap_or([&](std::string error) {
        RCLCPP_INFO_STREAM(node->get_logger(), "Transform lookup error: " << error);
      })
      .then([](sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg,
               const geometry_msgs::msg::TransformStamped &transform_to_map) {
            /// This callback gets called once the transform (target_frame="map", source_frame=point_cloud_msg->header.frame, time=point_cloud_msg->header.stamp) becomes available, the transform_to_map is this transform.
            
      });
```

See also the [TF synchronization example](../../icey_examples/src/tf_sychronization_example.cpp).

You will have to do however the actual transformation of the data (i.e. rigid transformation of the point cloud) yourself (PR are welcome to improve this!)

# Subscribing to single transforms 

When working with transforms between coordinate systems, you don't always need to request the transform at a specific time. Instead, you can subscribe to a single transform between two coordinate systems and receive each time it changes: 

```cpp 
node->icey()
      .create_transform_subscription("map", "base_link")
      .unwrap_or([&](std::string error) {
        RCLCPP_INFO_STREAM(node->get_logger(), "Transform subscriber failed: " << error);
      })
      .then([&](const geometry_msgs::msg::TransformStamped &new_transform) {
        Eigen::Matrix4d tf_mat = tf2::transformToEigen(new_transform.transform).matrix();
        RCLCPP_INFO_STREAM(node->get_logger(), "Received a new transform:\n" << tf_mat);
      });
```

See also the [TF subscription example](../../icey_examples/src/tf_subscription.cpp).
