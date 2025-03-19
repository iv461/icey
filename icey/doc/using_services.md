# Services 

Services are implemented using async/await syntax and service servers also allow to use asynchronous callback functions,  i.e. coroutines. 
The enables more possibilities that were previously not possible with regular ROS:

# Server 

To create a service server, you use your usual `create_service` function, which returns the response: 

```cpp
node->icey().create_service<ExampleService>(
      "set_bool_service", [&](auto request) -> Response {
        /// Build response 
        auto response = std::make_shared<ExampleService::Response>();
        return response;
      });
```

This example used a synchronous callback, that returns imeditely the response. 
The novelty of ICEY is that we can also use *asynchronous* callbacks, i.e. coroutines:

```cpp
/// Create a service client for an upstream service that is actually capable of answering the
/// request.
auto upstream_service_client =
      node->icey().create_client<ExampleService>("set_bool_service_upstream");

node->icey().create_service<ExampleService>(
      "set_bool_service", [&](auto request) -> icey::Promise<Response> {
        /// Call the upstream service with 1s timeout asynchronously:
        icey::Result<Response, std::string> upstream_result =
            co_await upstream_service_client.call(request, 1s);
        if (upstream_result.has_error()) {
          RCLCPP_INFO_STREAM(node->get_logger(),
                             "Upstream service returned error: " << upstream_result.error());
          /// Return nothing: This will simply not respond to the client, leading to a timeout
          co_return nullptr;
        } else {
          Response upstream_response = upstream_result.value();
          /// Respond to the client with the upstream response:
          co_return upstream_response;
        }
      });
```
This example calls another service inside the callback: This is an asynchronous operation that is awaited (`co_await`). Once it completes, the server sends the response. 
The difference between the synchronous callback is that the asynchronous one returns a `icey::Promise<Result>` instead of a `Result`. 

# Client 