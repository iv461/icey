# Using transforms (TF)

Coordinate system transforms are communicated in ROS via the `/tf` and `/tf_static` topics, so receiving them is inherently an asynchronous operation.
ICEY provides several ways to retrieve transforms: The usual `lookup` function (with async/await API), but also synchronizing a topic like a point cloud with a transform. ICEY even allows you to *subscribe* to a transform: A callback will be called every time a transform changes between two coordinate systems.

## Looking up transforms: 

To lookup a transform at a given time, you first create an`icey::TransformBuffer`: This is the usual combination of a subscriber on `/tf`/`/tf_static` and a buffer bundled into a single object. 

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

The difference is that in ICEY, the `lookup` call is asynchronous, while the original ROS API (`tf2_ros::Buffer::lookupTransform`) is synchronous -- it does *busy-waiting* with a separate executor.

ICEY, on the other hand, allows you to write *synchronous-looking* code using async/await, consistent with the APIs of other inherently asynchronous operations like service calls.

This difference is rather subtle, and shouldn't be too relevant if you're just a TF user.


## Synchronizing with a transform

Another way of obtaining transforms is to *declare* that you need a transform for each measurement time of another message like a point cloud. This is in contrast to the *imperative* style
where you explicitly request every single transform at a given time via a call to `lookup`.

With the declarative style you can essentially say *"I need this point cloud transformed in the map frame before further processing"* 

### Motivation 

To see why this is useful, we will analyze some common usage patterns of `lookupTransform'. 

1: Get the transform at the time of the measurement:

```cpp
void on_point_cloud(sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg) {
    auto measurement_time = point_cloud_msg->header.stamp;
    /// Get the pose of the LiDAR with respect to the map at the time this point cloud was measured:
    auto lidar_to_map = tf_buffer_->lookupTransform(point_cloud_msg->header.frame_id, map_frame_, tf2_ros::fromMsg(measurement_time), 200ms);
}
```

Other variants of this pattern (which are rather anti-patterns) are:

2: Ignore the header timestamp and just take the last transformation in the buffer to avoid waiting:

```cpp
void on_point_cloud(sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg) {        
    auto lidar_to_map = tf_buffer_->lookupTransform(point_cloud_msg->header.frame_id, map_frame_, tf2::TimePointZero, 200ms);
}
```

This assumes that the last transformation in the buffer is approximately at the same time as the message header time, an assumption that is generally unjustified.

3: Looking up at the current time in the callback: This is the time the message was *received* in callback, not the time the measurement *taken* (which may have been several milliseconds earlier):

```cpp
void on_point_cloud(sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg) {     
    /// Get the current time as this code (i.e. the callback) is executed:
    auto lidar_to_map = tf_buffer_->lookupTransform(point_cloud_msg->header.frame_id, map_frame_, this->get_clock().now(), 200ms);
}
```

However, all of these patterns have in common that we are interested in getting the transform for a time from another topic: We are *synchronizing* the transform with the messages of another topic.

### Usage with ICEY:

To synchronize a message (that has a header timestamp) with a transform in ICEY, you do:

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

This performs the synchronization using the `tf2_ros::MessageFilter`. 

You will have to do however the actual transformation of the data (i.e. rigid transformation of the point cloud) yourself (PR are welcome to improve this!)

## Subscribing to single transforms 

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
