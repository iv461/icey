# Build 


colcon build  --packages-select icey

# Debug 

```sh
ros2 run --prefix 'gdb -ex run --args' icey tf_approx_filter_example
```