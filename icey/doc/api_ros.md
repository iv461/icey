# ROS-related

Creating and spinning nodes:

```{doxygentypedef} icey::Node
```

```{doxygentypedef} icey::LifecycleNode
```

```{doxygenfunction} icey::create_node
```

```{doxygenfunction} icey::spin
```

The context owns all the Streams and is the Node-API visible when calling `node->icey()`:

```{doxygenclass} icey::Context
```

```{doxygenclass} icey::NodeWithIceyContext
```


```{doxygenclass} icey::NodeBookkeeping
```

```{doxygenstruct} icey::TFListener
```

```{doxygenstruct} icey::NodeInterfaces
```

