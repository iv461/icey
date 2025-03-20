.. icey documentation master file, created by
   sphinx-quickstart on Tue Feb 11 00:34:05 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

ICEY documentation
==================

ICEY is a a new API for the Robot Operating System (ROS) 2 that uses modern asynchronous programming with Streams and async/await syntax. It makes the asynchronous data-flow clearly visible and simplifies application code. It enables fast prototyping with less boilerplate code.

It is fully compatible to the ROS 2 API, it does not reinvent anything and supports all major features: parameters, subscribers, publishers, timers, services, clients, TF pub/sub. It supports not only regular nodes but also lifecyle nodes with a single API. 

ICEY operates smoothly together with the  `message_filters` package, and it uses it for synchronization. ICEY also allows for extention, demonstated by the already implemented support for `image_transport` camera subscriber/publishers.

It offers additional goodies such as:
   - Automatic bookeeping of publishers/subscribers/timers so that you do not have to do it 
   - No callback groups needed for preventing deadlocks -- service calls are always asynchronous
   - Handle many parameters easily with a single parameter struct that is registered automatically using static reflection, so that you do not need to repeat yourself

ICEY supports ROS 2 Humble and ROS 2 Jazzy.

The [icey_examples](../../icey_examples) package contains over one dozen of different example nodes, demonstrating the capabilites of ICEY.

Table of Contents
^^^^^^^^^^^^^^^^^
.. toctree::    
   :caption: Basics
   :maxdepth: 2
   
   getting_started
   first_icey_node
   publish_subscribe
   async_flow
   filtering
   coroutine_basics
   using_services
   using_tf
   parameters
   lifecycle_nodes
   translating_an_autoware_node

.. toctree::    
   :caption: Advanced
   :maxdepth: 2
   
   extending_icey
   api_reference
   development 
