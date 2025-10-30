# Asynchronous primitives

The asynchronous primitives like `Promise` and `Stream` are the building blocks of ICEY.

```{doxygenstruct} icey::Ok
```
```{doxygenstruct} icey::Err
```
```{doxygenstruct} icey::Result
```
```{doxygenstruct} icey::Nothing
```

```{doxygenclass} icey::Stream
```

```{doxygenclass} icey::PromiseBase
```

```{doxygenclass} icey::Promise
```

## Actual implementation of the stream (impl Stream)

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

```{doxygentypedef} icey::Clock
```
```{doxygentypedef} icey::Time
```
```{doxygentypedef} icey::Duration
```
