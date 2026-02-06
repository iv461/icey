# ICEY Examples

Examples on how to use the ICEY library. 

They demonstrate:

- Using the icey::Context for both regular and lifecycle Nodes
- Synchronous-looking service calls using async/await 
- Actions using async/await
- Simple publisher/subscriptions, demonstrating Streams
- Automatic synchronization between multiple topics 
- Looking up transforms using async/await
- Subscribing single transforms
- Synchronizing topics with transforms 
- Using parameter structs, greatly simplifying declaration of many parameters
- Transform broadcasting
- Using image transport subscriptions/publishers

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
ros2 run icey_examples service_server_example --ros-args -p service_name:=set_bool_service
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

# Run asynchronous service server example: 

Start the asynchronous server: 

```sh
ros2 run icey_examples service_server_async_await_example
```

And the upstream server:

```sh
ros2 run icey_examples service_server_example --ros-args -p service_name:=set_bool_service_upstream
```

And now the service client: 

```sh
ros2 run icey_examples service_client_async_await_example
```

You should now receive responses and see how both servers receive requests.

# Actions using async/await 

Run action server:


```sh 
ros2 run icey_examples action_server_async_await_example
```

And the action client:

```sh 
ros2 run icey_examples action_client_async_await_example
```


# TF lookup using async/await 

Start the subscription: 

```sh
ros2 run icey_examples tf_lookup_async_await_example 
```

And now the driver:

```sh
ros2 run icey_examples tf_pub_test_example 
```

The subscription should print regularly on every lookup.
 
# TF subscription example  

Start the TF broadcaster: 

```sh
ros2 run icey_examples tf_broadcaster_example 
```

Now the TF subscriber: 

```sh
ros2 run icey_examples tf_subscription_example 
```

It should print on every transform the transformation matrix: 

```
[INFO 1756324909.237720717] [icey_tf_subscription_example]: Received a new transform:
-0.446485 -0.894791         0       7.3
 0.894791 -0.446485         0       -73
        0         0         1         0
        0         0         0         1
``` 

# Run parameter struct example 

The parameter struct example shows how you can declare a lot of parameters without boilerplate code.

Runt the node:

```sh 
ros2 run icey_examples parameters_struct_example
```

Then inspect the parameters of the node from another terminal:

```sh 
ros2 param dump /icey_parameters_struct_example

/icey_parameters_struct_example:
  ros__parameters:
    amplitude: 3.0
    frequency: 10.0
    mode: single
    others:
      cov: []
      max_amp: 6.0
    qos_overrides:
      /parameter_events:
        publisher:
          depth: 1000
          durability: volatile
          history: keep_last
          reliability: reliable
    use_sim_time: false

```

You see all the parameter declared as well as the nested parameter (`others`)

If you change one parameter now: 
```sh 
ros2 param set /icey_parameters_struct_example amplitude 4.0
```

You see you get the notification: 

```sh 
[INFO 1740685079.892726882] [parameters_struct_example]: Parameter amplitude changed
```


# Lifecycle node example 

This example node spins and can be activated and deactivated. 

Run it: 

```sh
ros2 run icey_examples lifecycle_node_example
```

Then, from a second terminal deactivate it:

```sh
ros2 lifecycle set /lifecycle_node_example shutdown
```

And then activate it again: 


```sh
ros2 lifecycle set /lifecycle_node_example shutdown
```

