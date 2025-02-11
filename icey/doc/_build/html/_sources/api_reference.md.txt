# API Reference 

## Type aliases

```{doxygentypedef} icey::Node
```
```{doxygentypedef} icey::LifecycleNode
```

```{doxygentypedef} icey::Timer
```

```{doxygentypedef} icey::Parameter
```

```{doxygentypedef} icey::Clock
```
```{doxygentypedef} icey::Time
```
```{doxygentypedef} icey::Duration
```

## Functions 
```{doxygenfunction} icey::create_node
```
```{doxygenfunction} icey::spin
```
```{doxygenfunction} icey::spin_nodes
```
```{doxygenfunction} icey::declare_parameter_struct
```


## Fundamentals
```{doxygenstruct} icey::Nothing
```
```{doxygenstruct} icey::Result
```
## Streams 
These are all the available `Streams`, implementing subscribers, publishers/timers etc.


```{doxygenclass} icey::Stream
```

```{doxygenstruct} icey::ParameterStream
```

```{doxygenstruct} icey::Interval
```
```{doxygenstruct} icey::Set
```
```{doxygenstruct} icey::Validator
```

```{doxygenstruct} icey::SubscriptionStream
```
```{doxygenstruct} icey::TimerStream
```
```{doxygenstruct} icey::TimerImpl
```


```{doxygenstruct} icey::PublisherStream
```
```{doxygenstruct} icey::PublisherImpl
```

```{doxygenstruct} icey::ServiceStream
```

```{doxygenstruct} icey::ServiceClient
```
```{doxygenstruct} icey::ServiceClientImpl
```

```{doxygenstruct} icey::TransformSubscriptionStream
```
```{doxygenstruct} icey::TransformSubscriptionStreamImpl
```

```{doxygenstruct} icey::TransformPublisherStream
```

```{doxygenclass} icey::StreamImplDefault
```

### Actual implementation of Stream 

```{doxygenclass} icey::impl::Stream
```

## Filters 

The following filters handle synchronization, serialization  timeout and other operations on streams. 

```{doxygenstruct} icey::SimpleFilterAdapter
```

```{doxygenclass} icey::SynchronizerStream
```

```{doxygenstruct} icey::SynchronizerStreamImpl
```

```{doxygenstruct} icey::TF2MessageFilter
```

```{doxygenstruct} icey::TF2MessageFilterImpl
```

```{doxygenstruct} icey::TimeoutFilter
```

## Context
The context owns all the Streams. 

```{doxygenclass} icey::Context
```

## ROS-related 

```{doxygenclass} icey::NodeBookkeeping
```

```{doxygenstruct} icey::NodeInterfaces
```

```{doxygenstruct} icey::TFListener
```

```{doxygenclass} icey::NodeWithIceyContext
```

## C++20' coroutines support 

TODO 

## Traits 
```{doxygentypedef} icey::obs_err
```
```{doxygentypedef} icey::obs_val
```
```{doxygentypedef} icey::obs_msg
```
# Support for `image_transport`

```{doxygenstruct} icey::ImageTransportSubscriber
```
```{doxygenstruct} icey::ImageTransportSubscriberImpl
```

```{doxygenstruct} icey::ImageTransportPublisher
```


```{doxygenstruct} icey::CameraSubscriber
```
```{doxygenstruct} icey::CameraSubscriberImpl
```

```{doxygenstruct} icey::CameraPublisher
```

```{doxygenfunction} icey::create_image_transport_subscription
```

```{doxygenfunction} icey::create_image_transport_publisher
```

```{doxygenfunction} icey::create_camera_subscription
```
```{doxygenfunction} icey::create_camera_publisher
```