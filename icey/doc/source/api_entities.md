# Entities 

All the ROS entities (subscriptions, publishers, timers etc.) are wrapped so that they are either `Stream`s or provide an asynchronous interface with `icey::Promise` (service client, TF).


```{doxygenstruct} icey::ParameterStream
```

```{doxygenstruct} icey::SubscriptionStream
```
```{doxygenstruct} icey::TimerStream
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


## Parameter validation 

Some extra classes related to parameters:

```{doxygenstruct} icey::Interval
```
```{doxygenstruct} icey::Set
```
```{doxygenstruct} icey::Validator
```

```{doxygenstruct} icey::ValueOrParameter
```



