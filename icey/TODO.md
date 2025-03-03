# TODO 

## Must-have  for 0.1

Sorted by decreasing priority. 

- [ ] Test installing in Docker base image regarding dependencies 

- [X] Fix synchronize with transform and make it the main thing. 

- [ ] Service Client pending request is not cleaned up, it is only cleaned up
- [ ] Service call has no timeout, only on discovery it has a timeout

- [ ] Service is not a Stream: we cannot await streams, this only works by coincidence because the callback calls all the user-callbacks currenty. Co-await would access the result object after it has been transmitted to the receiver.

- [ ] Support async service server: return tx/rx channels 

- [ ] Test calling service in service server (like Tokio intro)
- [ ] Test parameter as value for TF sub


- [] Async/await: We maybe need a "Stream was closed" concept: Streams that are generally driven 
by ROS entities will never yield something regardless of how long we spin the ROS executor if the underlying ROS entity driving them was stopped. For example if the ROS-timer was cancelled. Or the subscription destroyed. In such a case, calling co_await on such streams would hang forever. We need to return None in this case or an extra end-of-Stream identitier (like tokio).

- [ ] Docs: Explain lambda-ownership, that lambdas need to be copied inside since the lifetime of the Stream is till the program exists. And that lvalues are copied as well. Think about whether it's good idea to force the user to explicitly mode the lambda inside so that the a named lvalue-lambda cannot be called by any other means.

- [ ] Docs: Explain synchronization 
- [ ] Docs: Explain Result-type for error handling 
- [ ] Docs: Explain that Result-type does not catch C++ - exceptions by default 
- [ ] Docs: Up-to-date extention tutorial
- [ ] Docs: Coroutines: add note that coroutines might fail to deliver value if the spin-loop is interrupted by Ctrl

- [ ] Benchmark perf and measure overhead compared to plain ROS to avoid surprises

- [ ] We still got a crash in service_client_async_await_example

- [ ] Do not accept Streams with Errors in filters that need to throw a new error: This should be a compile-time error, forcing the user to first handle the error

- [ ] Document how to access the internal ROS stuff in case it is needed, e.g. queue of syncher -> for this, after initialize callback is needed.

- [ ] Consider mergins NodeBookkeeping and Context: We already hold the shared poitner to timers and publishers in the Stream impl. Since stream impls are held by the Context, this already makes sure they live for as long as the node. So we would only need to hold stuff that is present once like a TF broadcaster in the context. By using auto node as the first argument, we could actually solve the cyclic dep

- [ ] `delay` with 

- [ ] Make first argument source_frame of subscribe_to_transform optional and then make a single synchronization function 

- [ ] Do not use the TF2 filter but instead make the TF 2 subscriber more flexible. The only reason we need the TF2 message filter is that we might do not know the source frame and want to read it from the message header. But the TF2 message filter does excessive locking and is not equivalent to manually looking up


- [ ] Add static asserts for the any filter that all the streams have the same value
- [ ] Add static asserts for the unpack transform that the stream holds a tuple
- [ ] Improve compile error when passing wrong callback signature -> std::invocable does not yield good ones 

## Error-handling

## Examples 

- [ ] Port a small autoware (or nav2) node as a proof that everything can be written using ICEY and to find out how many line of code we save

## Other nice-to-have features, not for 0.1

- [ ] Look into rclcpp::AsyncParametersClient, may be better suitable for the Parameter struct
- [ ] rclcpp also has a TimerInfo (previously called TimerEvent) with the time, use it as the state. 

- [ ] Remove use of RTTI in interpolateble stream 
- [ ] Pass error through synchronizers -> for this return Result from Interpolatables
- [ ] Allow chaining approx-time synchronizer with e.g. reference synchronizer by implementing averaging of all the header stampls of the tuple
- [ ] Automatic adaption of queue size in ApproxTimeSync
- [ ] Inputs requiring a parent stream should take it in the constructor 
- [ ] Maybe generalize concept of push/pull Stream 
- [ ] Add static asserts that message has header stamp for better compiler error messages
- [ ] Static_assert for the lambda signature 
- [ ] Support better parameter API: icey::Interval(0, 5.5) (i.e. determine the common type between the int and double literal) and allow for icey::Set("normal", "pulse", "single"), i.e. determine the common type of fixed-size char arrays correctly as std::string. 
- [ ] Allow std::array as parameter type with automatic validation for the size -> generally, add parameter type converters.

- [ ] In case we have overhead on calling callbacks, use the pmr::mem_pool allocator that acts like a linear allocator in case all Streams are equally large so that we achieve less cache misses.

- [] [Stream] member-then with static alloc idea: Return the state from the lambda conditionally on param (auto param that can be constexpr in C++17) -> p2300 does pipe then

- [X] Promise: Variant ErrorValue to be able to handle multiple errors in one `except` block. Needed because we can cascade thens with different ErrorValue types. -> not for 0.1 -> no cascading, we instread require that th input is error-free 

- [ ] Maybe support cascading the synchronizers -> not for 0.1

- [ ] Auto-pipelining with TBB graph

- [] Bus names: When returning multiple things from a callback, we can use strings instead of indices to unpack everything by index. (credit Bene) Possible implementation: another argument to then or Wrap the function in a NamedFunction("mybus", lambda). We coul even use hana::map to ensure at compile time that only existing names are looked up (That was the events  demo from Louis' talk at cppcon 2017)

- [ ] Message converters to subscribe directly to https://github.com/ros2/examples/blob/rolling/rclcpp/topics/minimal_publisher/member_function_with_type_adapter.cpp, 
- [ ] See also https://github.com/CursedRock17/ros2_common_type_adaptations
- [ ] Existing converters for pointcloud and Image to `cv::Mat`: https://github.com/roncapat/ros2_native_adapters
- [ ] publishing scalar values directly is implemented in autoware quite competently, re-use it: https://github.com/autowarefoundation/autoware.universe/blob/main/common/autoware_universe_utils/include/autoware/universe_utils/ros/debug_publisher.hpp#L33

- [ ] Maybe Simulink-style blocks, i.e. constant, step, function etc.
- [ ] Actions ? See https://github.com/BehaviorTree/BehaviorTree.ROS2/tree/humble

## API elegance/clarity

- [] API cleanup: we should have icey::Parameter instead of icey::ParameterStream, but we should rather rename either everything or nothing 
- [] API cleanup: Remove publish_transform, instead use simply publish, detect by value type whether we need to publish over the tf broadcaster.
- [ ] Parameters struct: Inconsistent API, why is it not a stream ? -> to not have to call .value() on it all the time

## Examples

- [ ] Kalman filter: Multiple inputs, interpolate 
- [ ] SLAM: Front-end and backend, running asynchronously
- [ ] Controller like PPC, gets position and velocity, etc. 

## Bugs/Leaks

- [X] Support official message filters API
- [X] Ensure soundness of our promises, see e.g. the rules for monads: https://stackoverflow.com/a/48553568 -> will satisfy Promise-spec instead, fallthrough etc.

## Done 

- [X] Code clean-up and dup: Wrap API in tuples, create graph afterwards
- [X] Code clean up: take shared_ptr by value everywhere
- [X] Forbid and detect cycles in the DAG: React has a "cyclic dependency detected" thing
- [X] Support ApproxTime synchronization
- [X] Writable publiser, otherwise no hardware drivers possible !
- [X] TOPOLOGICAL SORT DFG -> for the update-problem to happed a multiple input node is needed. This is only possible with fuse, that already outputs if at least one input updates -> not a problem
- [X] Forbid and detect creating new states in callback
- [X] Prevent getting the value from an stream, i.e. from the object SubscriptionStream,  it is read-only
- [X] Disallow graph change after node is created. Detect it. Provide instead "Node enable" functionality if someone wants to disable a publisher/subscriber
- [X] Explicit DAG
- [x] Allow for shared-ptr messages for perf, i.e. not copying the whole message but just notifying it changes. For this we need to just strip the ptr when calling node->subscribe<Msg>
- [X] Timer signal: like a signal, but a timer ! Basis for every signal-generator AND and the same time can be used as a timer. Commonly, one wants to publish something periodically.
- [X] "Waiting currently on" verbose printing for filters to see what's happening
- [X] Make sure we are moving all lambdas so that the captured values are moved as well !
- [X] Global variable: Check linkage issues regarding ODR: What happens if icey is included multiple times in the translation unit ? -> we likely need 
- [X] Correct initialization sequence: FIRST create all subscribers, then all publishers !!
- [X] TF Buffer 
- [x] Class-based API since OOP is popular and required of components 
- [x] Services
- [X] Private topic publishing -> maybe like python: if ~ in front of the topic name, then its private
- [X] Allow accessing the node: For logging and for all other objects
- [x] Pass timer to the callback to be able to implement one-off-timer (https://github.com/ros2/demos/blob/rolling/demo_nodes_cpp/src/timers/one_off_timer.cpp)
- [X] Rename to subscriber and publisher
- [X] create_publisher accepting input as argument, consistent with service client API 
- [X] Service client accepting input as argument using timer and different callback groups: https://docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html
- [X] Look at https://github.com/ros-controls/control_toolbox
- [X] Look at https://github.com/PickNikRobotics/RSL, maybe for monad, or for https://github.com/PickNikRobotics/RSL/blob/main/include/rsl/parameter_validators.hpp
- [X] Look at https://control.ros.org/rolling/index.html how to write controllers
- [X] Look at https://github.com/BehaviorTree/BehaviorTree.ROS2/tree/humble, maybe some ideas, but unlikely 
- [X] There are other filter libraries like https://github.com/ros/filters/tree/ros2, but not really used and only for simplistic tasks
- [x] How to implement RCLCPP_COMPONENTS_REGISTER_NODE ? -> solved by class-based API
- [X] Multiple nodes composition in the same process using function API 

- [X] Fully featured parameters
- [X] Allow returning `None` to emulate early return control-flow, otherwise we cannot stop data-pipeline. This is very common, i.e. checking message validity
- [X] TF Broadcaster
- [X] Publishing multiple outputs, i.e. single input, multiple output -> solvable with multiple thens
- [X] [functional API] `onCleanup`, `beforeInit()` and `afterInit` function to be able to call cleanup of external libraries , e.g. ONNX runtime (that would normally be the Node's destructor)
- [X] Partial dynamic graph: Use parameters as values for the arguments of other components like frame names. 
- [X] We need a after params callback in functional API to initialize parameters, otherwise imposible to init algorithms
- [X] Multiple-input single output-> serialize filter for modeling calling publisher from multiple places

- [X] Transform an existing message, i.e. output [it already transformed](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Using-Stamped-Datatypes-With-Tf2-Ros-MessageFilter.html). API: .transform("target_fram") ? -> not doing, in plain ROS it is not transformed as well

- [X] some autoware node needs dynamic parameter update callback to be set after algorithm has been initialized: https://github.com/autowarefoundation/autoware.universe/blob/main/planning/autoware_path_optimizer/src/node.cpp#L153 (does not make any sense to me) -> does not make sense
- [X] add custom loggers in the node: https://github.com/autowarefoundation/autoware.universe/blob/main/planning/autoware_path_optimizer/src/node.cpp#L161 -> easilly doable by mixing existing API
- [X] publish if subscribed (unnecessary imo) -> PublisherOptions, not for 0.1
- [X] be able to publish a tuple of optional values where some are set and some not. We need a way to directly publish some things and some not. for this, refactor connect using then, for this refactor then to receive a parent of the right type
- [X] Automatic synchronizer: idea is that it would be nice if there is only one icey::Sync method iIinterpolates if some signals are interpolatable, otherwise use approx time. If using approx time, exact time matches if the stamps match exactly. If the message type has no stamp, take receive time automatically. The idea is that the user should not specify which filter should be used, since we can infer this.
- [X] fuse is a composed filter, not elementary. it can be composed using SyncWithReference(any(signals), signals) -> removed fuse
- [X] Vector publisher: timings etc, vector_publish(sig1, sig2, ... , topic1, topic2 ...) -> won't fix

- [X] Graph engine: notify leaves
- [X] LookupTransform for TF Subscriber via prototypical Interpolatable, otherwise TF sub is useless ! -> depends on understanding hana and implementing `synch_with_reference`
- [X] publish shared ptr should be supported  
- [X] `SyncWithReference` filter -> one stream drives, from the others we get the last value. (basis for throttling), like Matlab's `synchronize`, mostly for constant frequency publishing driven by timers
- [X] Promise-API: .catch(), finally mostly important for timeout detection of service call 
- [X] ->then() as member

- [X] Implement Promises properly, adhering to fallthrough
- [X] Service client implementation  https://github.com/ros2/examples/blob/rolling/rclcpp/services/async_client/main.cpp#L97
- [X] Unit-tests GraphEngine: topo order should lead to correct single-update behavior -> ditched, not for 0.1
- [X] Unit-test promise behavior, fall-through etc. 
- [X] .catch() promise fo TF buffer, would be useful to print the lookup error
- [X] Remove MP11 as dependency
- [X] Service: fix soundness issue of the DFG, we store request and response inside the same node.
- [X] Re-throw exceptions if the promise holds an object of type exception as an ErrorValue and if no promise rejection handlers are registered.
- [X] Support Custom subscriber/publisher objects (with global state), mostly image_transport -> isn't a simple argument "subsriber type" enough ?

- [X] Automatic creation of callback groups for timer->client sequence ! otherwise deadlock ! (only if we support client/service) -> see maybe client example in nav2_stack -> https://docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html this was only for synchronous call. We do not need to create the callback groups if we are only using the async_call
- [X] Service client as member function, `call_service`
- [X] Timeouted Service request cleanup (automatic, periodically ? also use the available API functions to ensure there is nothing left)
- [X] Lifecycle nodes -> Template for the base class, sub/pub are essentially the same, maybe get the Nav2 wrapper -> we should not make the impression they are not supported. Generally, we have to use everywhere `rclcpp::NodebaseInterfaces`, another issue is that image_transport does not support LifeCycleNodes: https://github.com/ros-perception/image_common/pull/304

- [X] Code simplicity: consider holding the baggage that is currently in the ROSNode wrapper in the Context. Right now we essentially have two contexts.

- [X] Test with clang and build with ASAN(`-DCMAKE_CXX_FLAGS=-fsanitize=address`). -> unusable because ROS triggers new-delete-mismatch
- [X] Image-transport pub [is common](https://github.com/autowarefoundation/autoware.universe/blob/main/perception/autoware_tensorrt_yolox/src/tensorrt_yolox_node.cpp#L111)


- [X] Forbid subscribing to the same topic that is published by the same node -> not doing, resp of the user
- [X] About the premise that we only ever need transforms at the header time of some other topic: there is even a ROS tutorial [how to look up arbitrary times](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Time-Travel-With-Tf2-Cpp.html), but as I suspected it deals only with a constant delay, like 5 seconds. We could acutally support this by a Delay-node (Simulink style). We delay the sensor message and then lookup the TF (output maybe without delay if we assume we can receive old meesage). API maybe .delay(time)
- [X] A way to enable/disable the node -> Lifecycle node

- [X] Maybe support extention point, pass the Stream template arg with a default (i.e. for printing a warning that a parameter could not be retrieved) -> we already have with 

- [X] Fix segfault on termination -> cannot reproduce with gdb, seems like a bug in rclcpp. Our destruction order remains correct despite global var, so currently no idea about the root cause. -> Looks ugly and, so kind of important -> pass `handle SIGINT noprint nostop pass` to gdb (fixed, root-cause was that the rclcpp context global var gets destroyed before our global context gets destroyed)

- [X] Dynamic reconfigure without code-gen using boost hana (it can serialize structs) 
- [X] `unpack` tuple of obs to multiple obs, this is easy 
- [X] [Async/Await] Enable not having to allocate Streams dynamically to enable `async` via coroutines. Needed because we have no control over the allocation 
- [X] Prevent having to use an arrow -> only because everything needs to be reference-counted: Wrap the smart-ptr inside an object, i.e. use PIMPL. -> difficult, no solution without much code dup yet. Either pimpl or allow copying the objects
- [X] Fix segfault on termination with service example
- [X] [Async/Await] `await`: waitning to be able to write code that looks synchronous 
- [X] [Async/Await] `async`: allow async routignes using C++20 coroutines
- [X] `get_promise`-API needed intead of this->stream_ -> `impl`

- [X] People still like to check whether there [are subscribers on a topic](https://github.com/autowarefoundation/autoware.universe/blob/main/perception/autoware_tensorrt_yolox/src/tensorrt_yolox_node.cpp#L125) -> maybe "lazy" parameter on publish() ?  -> document how to access publisher 

- [X] Use Result https://github.com/bitwizeshift/result
- [X] Use fplus -> Hana supports enough FP
- [X] More FP
- [X] Use boost hana instead fo rolling our own bag of meta-programming tricks
- [X] Look at https://github.com/xhawk18/promise-cpp

- [X] Fix all warnings, some reorderings are left, and also the incomplete type of Context 
- [X] `timeout` filter
- [X] Rename Stream to Stream

- [X] Examples in separate package `icey_examples` -> TEST WHETHER WE CAN DEPEND ON THE ROS Package
- [X] https://github.com/ros-perception/imu_pipeline

- [X] PCL isn't ported yet: https://github.com/ros-perception/perception_pcl/issues/225 -> so won't fix
- [X] Maybe publish named-tuple, could be beneficial for many debug-publishers, i.e. return icey::named_tuple({"debug/cb", tic_time}, ...) -> publish(); 
- [X] Code: The ROS-Streams only need to write to the contained StreamImpl. For this, they should never capture this ! This way, we can pass them always by value since the internal StreamImpl won't be copied.

- [X] Scout ROS 2 demos for missing features: https://github.com/ros2/demos
- [X] Scout https://github.com/orgs/ros-perception/

- [X] Parameters struct: Groups, support nested structs like for AW's NDT hyper-params
- [X] Parameter struct: Allow plain (unwrapped) types, i.e. string, double etc.

- [X] Code cleanup: Pass state to Stream callback 
- [X] Do not leak the `NodeAttachable` implementation detail when users extend ICEY by proving defaults for the derived implementation. 
- [X] [Refactor] Simplify functional API, remove deffered attachment to ROS 

- [X] Parameter-struct refactoring: Use icey::Parameter for constrained types
- [X] Implement parameter validators
- [X] Doxygen parsable comments
- [X] API-docs

- [X] Consider removing the filters, parameters etc out of the Context and moving them to separate files. Then, create a class_based_api that simply defines the Node and wraps the free-functions synchronize and passed them the this->context instead of the global context. -> Done by removing functional API, class-based API is still called Context. Difficult to split into different files because we cannot extend an already defined class
- [X] Decide on whether streams should be default-constructable to be able to store them as members -> they already are
- [X] API cleanup: Rename "obs_msg", "obs_val" etc to smth more meaningful 
- [X] Rename parent to input

- [X] Consider renaming resolve to put_value and reject to put_error. A new method that gets state and sets it to none afterwards can be called "take"

- [X] tf2_ros Message filter: Just another filter: https://github.com/ros-perception/imu_pipeline/tree/ros2/imu_transformer
- [X] `StaticTransformBroadcaster` -> low prio since even the official do mentions you should use the executable instead of writing this code yourself: https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Cpp.html#the-proper-way-to-publish-static-transforms

- [X] Comment each line, do the icey-specific part ourselves, the rest can be done by LLMs. Everything output by LLMs is checked for factual accuracy of course.
- [X] Consider using `tf2_ros::AsyncBufferInterface::waitForTransform` for an own filter. But it only notifies once for an requested stamp, i.e. it is only a [promise](https://github.com/ros2/geometry2/blob/humble/tf2_ros/src/buffer.cpp#L240), not a stream.

- [X] `filter`: Pass through messages by binary predicate, document use-case of [validating messages](https://github.com/ros-navigation/navigation2/blob/main/nav2_util/include/nav2_util/validate_messages.hpp)

- [X] .buffer(N): Basis for `delay`-filter
- [X] Unit-Test context: does it create everything ? 

- [X] Moving lambdas: Make sure we do not have the same bug: https://github.com/TheWisp/signals/issues/20, add tests 

- [X] Unit-test that service client-server example driven by timer
- [X] Add static asserts everywhere in the public API, detect if it is Stream and detect callback signature, compiler messages are hard to understand otherwise -> Fixed by using Stream concept


- [X] Publisers do not get destroyed because the streams hold them and the streams have circular references. Do not capture strongly the impl::Streams. return weak_ptr from impl() and do not capture in done-handler

- [X] Async/await: In case the executor is stopped with Ctrl+C, the steam does not have a value but still we are trying to return it. This means, we would generally have to return a Result from await_resume. Problem is, this gives us an ugly syntax because C++ unlike Rust does not have pattern matching. In Rust, you would do `while let Some(val) = stream.wait`, but the best you could do in C++ is `while(auto val = co_await stream)` and then you would have to access the maybe-value with `*val`.  This issue is quite annoying because I don't think it's a good idea to force the user to unwrap the value even if in 99.9% of cases there is a value, only because on 0.1% of cases there might not be a value. Since the case there might not be any value happens only when pressing Ctrl+C while spinning, I think it would be better to just do what a ROS-node would do normally in this case: call rclcpp::shutdown and stop.
- [X] Unit-test that the use-count of the all the shared-ptrs to the streams is 1 after destructing the context (mem-leak test)

- [X] Clarify behavior of parameters regarding default value. Undeclared/no default etc. 

- [X] Fix (potential) use-after-free bug in ApproxTimeSync -> we still shall never capture this !

- [X] Unit-test the synchronizers, is the lookupTransform correct ?
- [X] Unit-test all entities
- [X] Unit-test all filters
- [X] Unit-test parameter stream
