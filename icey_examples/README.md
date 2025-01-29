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
