# Services using async/await

One of the biggest novelties of ICEY is that it allows to use services with async/await syntax.
ICEY is the first library to provide such an API using the new C++20 coroutine feature.

ICEY also allows for service servers to use asynchronous callback functions,  i.e. coroutines which enables 
 a more powerful behavior like calling other services inside callbacks, something that was previously only difficult and clumsy to achieve with regular ROS [2, 3].

## Client 

Service clients call a service, and this is an inherently asynchronous operation -- we don't know when (or if) we will receive the response. 
What we want most of the time however is to continue doing other work only *after* we got the response. 

The regular ROS 2 API does not offer a synchronous API for calling services (meaning a function that calls the service and blocks until the response was received) -- instead the user has to manually spin the event loop which is error-prone because it leads to deadlocks when done inside a callback [1].  

With ICEY, waiting for the response becomes easy thanks to the async/await based API: 

```cpp
auto service = node->icey().create_client<ExampleService>("set_bool_service");

icey().create_timer(1s)
    .then([this](size_t) -> icey::Promise<void> {
        /// Build a request each time the timer ticks
        auto request = std::make_shared<ExampleService::Request>();
        request->data = true;
        
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

You can call services and await the response inside any callback (timer, subscription, service server). You can implement synchronization of operations (*first* call service, *then* do x) while the underlying operations remain asynchronous. 

All of this is possible thanks to coroutines which allow to write __single-threaded__ asynchronous code. 

## Server 

To create a service server, you use your usual `create_service` function and pass it a callback that receives the request and returns the response: 

### Using synchronous callbacks
```cpp
node->icey().create_service<ExampleService>(
      "set_bool_service", [&](auto request) -> Response {
        /// Build response 
        auto response = std::make_shared<ExampleService::Response>();
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



# References 

- [1] Tutorial on how to use callback groups in rclcpp https://discourse.ros.org/t/how-to-use-callback-groups-in-ros2/25255
- [2] https://github.com/tgroechel/lifecycle_prac/blob/main/src/async_srv.cpp#L10-L69C1
- [3] https://github.com/ijnek/nested_services_rclcpp_demo
- [4] Discussion about async/await and proposal by William Woodall: https://github.com/ros2/ros2_documentation/issues/901