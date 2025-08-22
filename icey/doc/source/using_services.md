# Services using async/await

One of ICEY's most notable features is its ability to use services with  async/await syntax.
ICEY is the first library to provide this type of API using the new C++20 coroutine feature.

Calling a service returns a `Promise` that you can `co_await`. Additionally, every service call requires a timeout.
For service servers, ICEY allows to use asynchronous callback functions (i.e. coroutines) which enables call other services for example: This behavior was previously only achievable in a rather clumsy way [2, 3], so ICEY is more powerful in this regard.

With async/await, you also don't have to deal with callback groups to prevent deadlocks [1].

## Client 

Calling a service is an inherently asynchronous operation.
Most of the time we want to synchronize the call however, i.e. to continue doing other work only *after* we receive the response. 

The regular ROS 2 API does not offer a synchronous API for calling services -- instead the user must manually spin the event loop. This is error-prone since it leads to deadlocks when done inside a callback [1].  

With ICEY, waiting for the response becomes easy thanks to the async/await API: 

```cpp
auto service = node->icey().create_client<ExampleService>("set_bool_service");

icey().create_timer(1s)
    .then([this](size_t) -> icey::Promise<void> {
        /// Build a request each time the timer ticks
        auto request = std::make_shared<ExampleService::Request>();
        request->data = true;

        /// Call the service (asynchronously) and await the response with 1 second timeout:
        icey::Result<Response, std::string> result = co_await service.call(request, 1s);
        
        if (result.has_error()) {
            /// Handle errors: (possibly "TIMEOUT" or "INTERRUPTED")
            RCLCPP_INFO_STREAM(node->get_logger(), "Got error: " << result.error());
        } else {
            RCLCPP_INFO_STREAM(node->get_logger(), "Got response: " << result.value()->success);
        }
        co_return;
    })
```
See also the [Service client](../../../icey_examples/src/service_client_async_await.cpp) example.

### Every service call has a timeout 

Another advantage of ICEY is that it requires you to specify a timeout for each service call. The regular ROS API does not provide this option, so it cannot automatically clean up requests that were never answered. Instead, users must manually clean up requests when a timeout occurs. Otherwise, a memory leak will occur.

With ICEY you never need to do any kind of manual cleanup -- pending requests are cleaned up automatically if a timeout occurs. This implementation is similar to the one discussed in [4].

## Server 

To create a service server, you use your usual `create_service` function and pass it a callback that receives the request and returns the response: 

### Using synchronous callbacks
```cpp
node->icey().create_service<ExampleService>(
      "set_bool_service", [&](auto request) -> Response {
       
        auto response = std::make_shared<ExampleService::Response>();
        /// Build here the response
        return response;
      });
```

This example uses a synchronous callback, meaning it returns the response immediately.

### Using asynchronous callbacks

The novelty of ICEY is that we can also use *asynchronous* callbacks (i.e. coroutines) as service callbacks:

```cpp
/// Create a service client for an upstream service that is actually capable of answering the
/// request.
auto upstream_service_client =
      node->icey().create_client<ExampleService>("set_bool_service_upstream");

node->icey().create_service<ExampleService>(
      "set_bool_service", 
        /// An asynchronous callback (coroutine) returns a Promise<Response>:
        [&](auto request) -> icey::Promise<Response> {

        /// Call the upstream service with 1s timeout asynchronously:
        icey::Result<Response, std::string> upstream_result = co_await upstream_service_client.call(request, 1s);

        if (upstream_result.has_error()) {
          RCLCPP_INFO_STREAM(node->get_logger(),
                             "Upstream service returned error: " << upstream_result.error());
          /// Return nothing: This will simply not respond to the client, leading to a timeout
          co_return nullptr;
        } else {
          Response upstream_response = upstream_result.value();
          /// Respond to the client with the upstream server's response:
          co_return upstream_response;
        }
        
      });
```
See also the [Service server](../../../icey_examples/src/service_server_async_await.cpp) example.

This example calls another service inside the callback: This is an asynchronous operation that is awaited (`co_await`). Once it completes, the server sends the response. 
The difference between the synchronous callback is that the asynchronous one returns a `icey::Promise<Response>` instead of a `Response`. 

Generally, you can call services and await the response inside the callback of any ROS entity: timers, subscriptions, service servers.
For this, the callback must be asynchronous, i.e. must return an  `icey::Promise`.

# References 

- [1] [How to use callback groups in ROS2](https://discourse.ros.org/t/how-to-use-callback-groups-in-ros2/25255)
- [2] [Asynchronous response example](https://github.com/tgroechel/lifecycle_prac/blob/main/src/async_srv.cpp#L10-L69C1)
- [3] [Nested services demo](https://github.com/ijnek/nested_services_rclcpp_demo)
- [4] Discussion about async/await and API proposal by William Woodall: [https://github.com/ros2/ros2_documentation/issues/901#issuecomment-754167904](https://github.com/ros2/ros2_documentation/issues/901#issuecomment-754167904)