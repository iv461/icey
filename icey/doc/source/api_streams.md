# Entities 

All the ROS entities such as subscriptions, publishers/timers etc., most of them follow the `Stream` concept.

## Asynchronous primitives 

```{doxygenstruct} icey::Result
```
```{doxygenstruct} icey::Nothing
```

```{doxygenclass} icey::Stream
```

```{doxygenclass} icey::Promise
```


```{doxygentypedef} icey::Clock
```
```{doxygentypedef} icey::Time
```
```{doxygentypedef} icey::Duration
```

## Entities 

```{doxygenstruct} icey::ParameterStream
```

```{doxygenstruct} icey::ValueOrParameter
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

```{doxygenstruct} icey::ServiceStream
```

```{doxygenstruct} icey::ServiceClient
```

```{doxygenstruct} icey::TransformSubscriptionStream
```

```{doxygenstruct} icey::TransformBuffer
```

```{doxygenstruct} icey::TransformPublisherStream
```

```{doxygenclass} icey::StreamImplDefault
```

```{doxygenstruct} icey::WithDefaults
```

Actual implementation of the stream.

```{doxygenclass} icey::impl::Stream
```

## Traits 
```{doxygentypedef} icey::ErrorOf
```
```{doxygentypedef} icey::ValueOf
```
```{doxygentypedef} icey::MessageOf
```

## C++20 coroutines support 

```{doxygenstruct} icey::Awaiter
```
