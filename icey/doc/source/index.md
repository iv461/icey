# ICEY documentation

ICEY is a a new API for the Robot Operating System (ROS) 2 that allows for modern asynchronous programming using Streams, Promises and C++20 coroutines with async/await syntax. This simplifies application code and makes the asynchronous data-flow clearly visible. This enables fast prototyping with less boilerplate code.

It is fully compatible to the ROS 2 API and allows for gradual adoption since the `icey::Node` extends a regular ROS-Node. It supports all major features of ROS: parameters, subscriptions, publishers, timers, services, clients, TF. It supports not only regular nodes but also lifecycle nodes with a single API. 

ICEY operates smoothly together with the  `message_filters` package, and it uses it for synchronization. ICEY also allows for extension, demonstrated by the the support for `image_transport` camera subscription/publishers that is already implemented.

It offers additional goodies such as:
- Automatic bookkeeping of publishers/subscriptions/timers so that you do not have to do it 
- No callback groups needed for preventing deadlocks -- async/await allows for synchronously looking code while the service calls remain asynchronous
- Handle many parameters easily with a single parameter struct that is registered automatically using static reflection, so that you do not need to repeat yourself

ICEY supports ROS 2 Humble and ROS 2 Jazzy.

The [icey_examples](../../icey_examples) package contains many different example nodes, demonstrating the capabilities of ICEY.

```{toctree}
:caption: Basics
:maxdepth: 2

getting_started
first_icey_node
publish_subscribe
timers
parameters
async_flow
coroutine_basics
using_services
using_tf
lifecycle_nodes
```

```{toctree}
:caption: Advanced
:maxdepth: 2

extending_icey
api_reference
development
```