# Lifecycle nodes

ICEY supports lifecylce nodes, for this you simply need to: 

```cpp
auto lifecycle_node = icey::create_node<icey::LifecycleNode>(argc, argv, "planner_node");
```

See also the [lifecycle node example](../../icey_examples/src/lifecycle_nodes.cpp). 

The API for creating Streams remains the same regardless of whether you use a regular node or an lifecycle node. 
This is because the underlying `icey::Context`-type is the same.

This means that if you want to create functions that are generic and work for both regular nodes and lifecycle nodes, you can take as `icey::Context` as an argument:

```cpp
  void create_subscriptions(icey::Context &context) {
    context.create_subscription<sensor_msgs::msg::Image>("camera")
        .then(...);
    context.create_transform_subscription("map", "base_link")
        .then(...);
  }
```

This also means, you do need to use function templates that use the node as a template parameter.