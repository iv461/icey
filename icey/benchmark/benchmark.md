# Benchmarking ICEY: Measuring throughput, latency and CPU load

We performed the experiments on ROS 2 Humble on Ubuntu 22.04 LTS and rclcpp version 16.0.12.
RMW used is `rmw_cyclonedds_cpp` 1.3.4.
Linux 6.8.0-65-generic and perf version 6.8.12.
We measure on a laptop with an Intel i5-6300HQ CPU and 8GiB RAM.

## Draft: TF lookup 

Since ICEY does not use an extra executor for TF lookups, we hypothesize a lower CPU load. 

We measure the reference implementation using pure rclcpp `tf_lookup_async_ref.cpp` 
and compare it to the `tf_lookup_async_await.cpp` example that uses ICEY. Both examples are semantically equivalent. 

### Experimental procedure 

### Results 



## Events throughput

We measured the *event throughput*, that is a classic benchmark for event loops. It measures how many tasks, i.e. functions, also called events, can be dispatched per time period.
In ROS, we simply create a one-off Timer with zero seconds period time.
We tested for this:
- rclcpp [src/events_throughput.cpp](src/events_throughput.cpp)
- rclc [src/events_throughput.c](src/events_throughput.c)
- NodeJS [src/events_throughput_node.js](src/events_throughput_node.js)

### Results 

rclcpp has the lowest throughput at 20k, already failing to create many timers in a loop due to an algorithmic inefficiency bug (https://github.com/ros2/rclcpp/issues/2942). 
This bug is circumvented by using rclc, which achieves a higher throughput of 200k - 300k events/s with CycloneDDS RMW. 
With the zenoh RMW, the throughput is higher at around 900k events per second .
The highest throughput achieves NodeJS with over 2 million events per second.