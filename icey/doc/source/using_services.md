# Services 

Services are implemented using async/await syntax and service servers also allow to use asynchronous callback functions,  i.e. coroutines. 
This allows for a more powerful  behavior that was previously not possible with regular ROS.


## Client 

Service clients call a service service, and this is an inherently asynchronous operation -- we do not know when we will receive the response. 
What we want most of the time however is to continue doing other work only *after* we got the response. 

The regular ROS API does not offer a synchronous API for calling services -- instead it forces the user to manually spin the event loop. 

ICEY on the other hand provides a async/await- based API: 

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

Using that, you can call services and await the response inside any callback. This means, you can implement a synchronous operations while the underlying operations are still asynchronous. 

All of this is possible thanks to coroutines, they allow to write a __single-threaded__ asynchronous code. 


What the compiler does is something along the lines of: 

```cpp
          
//// Compiler creates a continuation
auto coro_continuation1 = [](icey::Result<Response, std::string> result) {
  if (result.has_error()) {
        /// Handle errors: (possibly "TIMEOUT" or "INTERRUPTED")
        RCLCPP_INFO_STREAM(node->get_logger(), "Got error: " << result.error());
    } else {
        RCLCPP_INFO_STREAM(node->get_logger(), "Got response: " << result.value()->success);
    }
};

node->create_wall_timer(1s, [this]() {
        /// Build a request each time the timer ticks
        auto request = std::make_shared<ExampleService::Request>();
        request->data = true;
        
        icey::Promise<Response, std::string> output_promise;                  /// <-- Compiler constructs the output promise
        service.async_send_request(request, [&result](auto future) {          /// <--- ICEY code
            if(future.valid()) {
              output_promise.set_value(future.get()); 
              continuation1(output_promise.get_state().get());
            } else {
              output_promise.set_error(rclcpp::ok() ? "TIMEOUT" : "INTERRUPTED");
              continuation1(output_promise.get_state().get());
            }
        }); /// Enqueues the call
    });
rclcpp::spin(node);
```


## Server 

To create a service server, you use your usual `create_service` function, which returns the response: 

### With synchronous callback
```cpp
node->icey().create_service<ExampleService>(
      "set_bool_service", [&](auto request) -> Response {
        /// Build response 
        auto response = std::make_shared<ExampleService::Response>();
        return response;
      });
```

This example used a synchronous callback, it returns the response immediately.

### With asynchronous callback

The novelty of ICEY is that we can also use *asynchronous* callbacks, i.e. coroutines:

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
This example calls another service inside the callback: This is an asynchronous operation that is awaited (`co_await`). Once it completes, the server sends the response. 
The difference between the synchronous callback is that the asynchronous one returns a `icey::Promise<Result>` instead of a `Result`. 

See also the [Service server](../../icey_examples/src/service_server_async_await.cpp) example.
