# Using transforms (TF)

Coordinate system transforms are communicated in ROS over the topic `/tf` and `/tf_static`, so receiving them is inherently an asynchronous operation.
To obtain transforms from TF, ICEY provides again an async/await based API. 

Additionally, ICEY allows you to *subscribe* to a single transform between to coordinate system instead of requesting a transform at a specific time. This features is also useful in some applications. 

# Looking up transforms: 

To lookup a transform at a specific time, you first create in ICEY a `icey::TransformBuffer`: This is the usual combination of a subscriber and a buffer bundled in a single object. 

You can then call `lookup` and await it: 

```cpp 
icey::TransformBuffer tf_buffer = node->icey().create_transform_buffer();

node->icey()
      .create_subscription<sensor_msgs::msg::PointCloud2>("/icey/test_pcl")
      // Use a coroutine (an asynchronous function) as a callback for the subscriber:
      .then([&tf_buffer, &node](sensor_msgs::msg::PointCloud2::SharedPtr point_cloud) -> icey::Promise<void> {

        auto tf_result = co_await tf_buffer.lookup("map", point_cloud->header.frame_id,
                                      icey::rclcpp_to_chrono(point_cloud->header.stamp), 200ms);
        /// Continue transforming the point cloud here ..
    });
```
See also the [TF lookup](../../icey_examples/src/tf_lookup_async_await.cpp) example.

The signature of the `lookup` function is the same as the `lookupTransform` function that you are used to. The difference is that in ICEY, we provide an async/await API, consistent with other *synchronous* APIs of inherently asynchronous operations (like service calls). 

This means, while the wait in the original ROS API is synchronous ("busy-wait"), with ICEY it is an *asynchronous* wait. 

The call to lookup returns a `icey::Promise`


# Subscribing to single transforms 


# Synchronizing and transforming data 

A common pattern of using transforms in ROS is 