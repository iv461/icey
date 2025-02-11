# API Reference 

## Streams 
These are all the available `Streams`, implementing subscribers, publishers/timers etc.


```{doxygenclass} icey::Stream
:project: icey
:members:
```

```{doxygenstruct} icey::ParameterStream
:project: icey
:members:
```

```{doxygenstruct} icey::SubscriptionStream
:project: icey
:members:
```

```{doxygenstruct} icey::PublisherStream
:project: icey
:members:
```

```{doxygenstruct} icey::ServiceStream
:project: icey
:members:
```

```{doxygenstruct} icey::ClientStream
:project: icey
:members:
```
```{doxygenstruct} icey::ClientStreamImpl
:project: icey
:members:
```

## Filters 

The following filters handle synchronization, serialization  timeout and other operations on streams. 

```{doxygensclass} icey::SynchronizerStream
:project: icey
:members:
```

```{doxygenstruct} icey::TF2MessageFilter
:project: icey
:members:
```

```{doxygenstruct} icey::TimeoutFilter
:project: icey
:members:
```

## ROS-related 

```{doxygenstruct} icey::TimeoutFilter
:project: icey
:members:
```

## Context
The context is responsible for 
```{doxygenclass} icey::Context
:project: icey
:members:
```

## C++20' coroutines support 
