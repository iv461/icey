# ICEY Examples

ROS Package that shows how to consume the ICEY library. 

We demonstrate how to: 

- Write simple publisher/subscribers with promises, chaining then
- Using C++20 coroutines to achieve synchronous-looking, async/await-style code 
- Automatic synchronization between multiple topics 
- Subscribing single transforms and looking them transforms by synchronizing with other topics 
- Parameter structs, greatly simplifying declaration and managing of multiple parameters
- Simple functional API for launching quickly multiple nodes using a single cpp-file 
- Service server and clients including chaining service calls without callback hell or risk of deadlocks
- Using lifecycle nodes by only changing the base class
- Transform broadcasting
- Using image transport subscribers/publishers
- Interoperability with the message_filters library




# Run service example 

Run the service client: 

```sh 
ros2 run icey_examples service_client_async_await_example
```

It should start printing: 

```
[INFO 1739536174.336721959] [signal_generator_async_await_example]: Timer ticked, sending request: 1
[INFO 1739536175.336992943] [signal_generator_async_await_example]: Service1 got error: SERVICE_UNAVAILABLE
[INFO 1739536176.337142245] [signal_generator_async_await_example]: Service2 got error: SERVICE_UNAVAILABLE
[INFO 1739536176.337245071] [signal_generator_async_await_example]: Timer ticked, sending request: 1
[INFO 1739536177.337368092] [signal_generator_async_await_example]: Service1 got error: SERVICE_UNAVAILABLE
[INFO 1739536178.337532123] [signal_generator_async_await_example]: Service2 got error: SERVICE_UNAVAILABLE
[INFO 1739536178.337611589] [signal_generator_async_await_example]: Timer ticked, sending request: 1
[INFO 1739536179.337728093] [signal_generator_async_await_example]: Service1 got error: SERVICE_UNAVAILABLE
...

```

Then, in another terminal, run the service server:

```sh 
ros2 run icey_examples service_server_example
```

Which should print:

```
[INFO 1739536197.338006812] [signal_generator_async_await_example]: Got response1: 0
[INFO 1739536198.337306617] [signal_generator_async_await_example]: Timer ticked, sending request: 1
[INFO 1739536198.337861565] [signal_generator_async_await_example]: Got response1: 0
[INFO 1739536198.338268818] [signal_generator_async_await_example]: Got response1: 0
[INFO 1739536199.337331935] [signal_generator_async_await_example]: Timer ticked, sending request: 1
[INFO 1739536199.337942713] [signal_generator_async_await_example]: Got response1: 0
[INFO 1739536199.338201448] [signal_generator_async_await_example]: Got response1: 0
[INFO 1739536200.337282427] [signal_generator_async_await_example]: Timer ticked, sending request: 1
[INFO 1739536200.337812212] [signal_generator_async_await_example]: Got response1: 0
[INFO 1739536200.337953596] [signal_generator_async_await_example]: Got response1: 0
```

And now the the service client prints:

```
[INFO 1739536197.337218652] [signal_generator_async_await_example]: Timer ticked, sending request: 1
[INFO 1739536197.337773922] [signal_generator_async_await_example]: Got response1: 0
[INFO 1739536197.338006812] [signal_generator_async_await_example]: Got response1: 0
[INFO 1739536198.337306617] [signal_generator_async_await_example]: Timer ticked, sending request: 1
[INFO 1739536198.337861565] [signal_generator_async_await_example]: Got response1: 0
[INFO 1739536198.338268818] [signal_generator_async_await_example]: Got response1: 0
...
```