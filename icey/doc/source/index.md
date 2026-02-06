# ICEY documentation

ICEY is a new client API for modern asynchronous programming in the Robot Operating System (ROS) 2. It uses C++20 coroutines with async/await syntax for service calls and TF lookups. ICEY allows you to model data flows based on streams and promises. These features simplify application code and make asynchronous data flows clearly visible.

### Problems ICEY solves:
 - Deadlocks are impossible since there is no need to manually spin the ROS executor (event loop) inside callbacks.
 - There are no memory leaks during service calls — every request is cleaned up automatically after the specified timeout.
 - All callbacks can be asynchronous functions (i.e., coroutines), which makes it possible to call and co_await other asynchronous operations inside callbacks.
 - A consistent async/await-based API for all asynchronous operations, including service calls and TF lookup.
 - Topic synchronization without boilerplate code

ICEY is fully compatible with the ROS 2 API since it is built on top of rclcpp. This allows for gradual adoption. It supports all major ROS features: parameters, subscriptions, publishers, timers, services, clients, and TF. Additionally, ICEY supports lifecycle nodes using a single API.
ICEY operates smoothly with the message_filters package, using it for synchronization. ICEY is also extensible, as demonstrated by its support for image transport camera subscription/publishers.
ICEY supports ROS 2 Humble and ROS 2 Jazzy.

The [icey_examples](../../icey_examples) package contains many different example nodes, demonstrating the capabilities of ICEY.

```{toctree}
:caption: Basics
:maxdepth: 2

first_icey_node
publish_subscribe
timers
parameters
async_flow
coroutine_basics
using_services
using_tf
using_actions
lifecycle_nodes
```

```{toctree}
:caption: Advanced
:maxdepth: 2

extending_icey
api_reference
development
```