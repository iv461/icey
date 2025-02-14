# Streams 

These are all the available `Streams`, implementing subscribers, publishers/timers etc.

```{doxygentypedef} icey::Clock
```
```{doxygentypedef} icey::Time
```
```{doxygentypedef} icey::Duration
```

```{doxygenstruct} icey::Nothing
```
```{doxygenstruct} icey::Result
```


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

```{doxygenstruct} icey::WithDefaults
```

Actual implementation of the stream.

```{doxygenclass} icey::impl::Stream
```

## Traits 
```{doxygentypedef} icey::obs_err
```
```{doxygentypedef} icey::obs_val
```
```{doxygentypedef} icey::obs_msg
```

## C++20 coroutines support 

```{doxygenstruct} icey::StreamCoroutinesSupport
```
