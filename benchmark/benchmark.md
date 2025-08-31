# Benchmarking ICEY: Measuring throughput, latency and CPU load

We performed the experiments on ROS 2 Humble on Ubuntu 22.04 LTS and rclcpp version 16.0.12.
RMW used is `rmw_cyclonedds_cpp` 1.3.4.
Linux 6.8.0-65-generic and perf version 6.8.12.
We measure on a laptop with an Intel i5-6300HQ CPU and 8GiB RAM.

## TF lookup 

Since ICEY does not use an extra executor for TF lookups, we hypothesize a lower CPU load. 

We measure the reference implementation using pure rclcpp `tf_lookup_async_ref.cpp` 
and compare it to the `tf_lookup_async_await.cpp` example that uses ICEY. Both examples are semantically equivalent. 

### Experimental procedure 

### Results 

