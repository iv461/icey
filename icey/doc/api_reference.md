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

## Streams 
These are all the available `Streams`, implementing subscribers, publishers/timers etc.


```{doxygenclass} icey::Stream
```

```{doxygenstruct} icey::ParameterStream
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

### Actual implementation of Stream 

```{doxygenclass} icey::impl::Stream
```

## Filters 

The following filters handle synchronization, serialization  timeout and other operations on streams. 

```{doxygensclass} icey::SimpleFilterAdapter
```

```{doxygensclass} icey::SynchronizerStream
```

```{doxygensclass} icey::SynchronizerStreamImpl
```

```{doxygenstruct} icey::TF2MessageFilter
```

```{doxygenstruct} icey::TF2MessageFilterImpl
```

```{doxygenstruct} icey::TimeoutFilter
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

## Context
The context owns all the Streams. 

```{doxygenclass} icey::Context
```

## C++20' coroutines support 

