# TODO 

## Must-have  for 0.1

Sorted by decreasing priority. 

- [X] Fully featured parameters
- [ ] TOPOLOGICAL SORT DFG -> for the update-problem to happed a multiple input node is needed. This is only possible with fuse, that already outputs if at least one input updates -> not a problem
- [X] Allow returning `None` to emulate early return control-flow, otherwise we cannot stop data-pipeline. This is very common, i.e. checking message validity
- [ ] LookupTransform for TF Subscriber via prototypical Interpolatable, otherwise TF sub is useless !
- [X] Publishing multiple outputs, i.e. single input, multiple output -> solvable with multiple thens
- [ ] Service client implementation  https://github.com/ros2/examples/blob/rolling/rclcpp/services/async_client/main.cpp#L97
- [ ] Automatic creation of callback groups for timer->client sequence ! otherwise deadlock ! (only if we support client/service) -> see maybe client example in nav2_stack
- [X] Multiple-input single output-> serialize filter for modeling calling publisher from multiple places
- [ ] Promise-API: .catch(), finally mostly important for timeout detection of service call 

- [] Unit-tests testing the node

## Error-handling 

- [ ] Forbid subscribing to the same topic that is published by the same node 
- [ ] Forbid and detect cycles in the DAG: React has a "cyclic dependency detected" thing
- [ ] Check callback has the right type, compiler messages are hard to understand 

## Other nice-to-have features

- [ ] .catch() promise fo TF buffer, would be useful to print the lookup error
- [ ] Support Custom subscriber/publisher objects (with global state), mostly image_transport -> isn't a simple argument "subsriber type" enough ?
- [ ] Support Single reference synchronozation (like Matlab's `synchronize`, mostly for constant frequency publishing driven by timers)
- [ ] Support ApproxTime synchronization
- [ ] [functional API] `onCleanup`, `beforeInit()` and `afterInit` function to be able to call cleanup of external libraries , e.g. ONNX runtime (that would normally be the Node's destructor)
- [ ] Partial dynamic graph: Use parameters as values for the arguments of other components like frame names. 
    Sometimes we want to use the values of parameters to initialize the other stuff, i.e. frames of tf (topics should not be needed). So we need to somehow first get the parameters and then update sub/pub 
- [ ] A way to enable/disable the node 
- [ ] Maybe Simulink-style blocks, i.e. constant, step, function etc.
- [ ] tf2_ros Message filter: Just another filter: https://github.com/ros-perception/imu_pipeline/tree/ros2/imu_transformer

## API elegance/clarity

- [ ] Dynamic reconfigure without code-gen using boost hana (it can serialize structs)
- [ ] .then() as member
- [ ] Prevent having to use an arrow -> only because everything needs to be reference-counted: Wrap the smart-ptr inside an object, i.e. use PIMPL. -> difficult, no solution without much code dup yet. Either pimpl or allow copying the objects
- SEE https://github.com/ros-navigation/navigation2/blob/humble/nav2_util/include/nav2_util/service_client.hpp
- [ ] Search for code that fixes the most common issues like setting int to an double param should be allowed

## Documentation 

- [ ] Up-to-date docs 
- [ ] Visualize DFG, maybe create a sequence diagram with websequencediagrams.com
- [ ] Doxygen parsable comments 

## Examples

- [ ] Maybe port a small autoware or nav2 node
- [ ] Kalman filter: Multiple inputs, interpolate 
- [ ] SLAM: Front-end and backend, running asynchronously
- [ ] Controller like PPC, gets position and velocity, etc. 

- [ ] Scout ROS 2 demos for missing features: https://github.com/ros2/demos
- [ ] Scout https://github.com/orgs/ros-perception/

## Bugs/Leaks

- [ ] Support official message filters API
- [ ] Really think about memory leaks, they should not be possible !
- [ ] Check reference counts on all shared ptr to understand whrere they are all incremented
- [ ] Ensure soundness of our promises, see e.g. the rules for monads: https://stackoverflow.com/a/48553568
- [ ] Actions ? See https://github.com/BehaviorTree/BehaviorTree.ROS2/tree/humble
- [ ] Lifecycle nodes

- [ ] Fix segfault on termination -> cannot reproduce with gdb, seems like a bug in rclcpp. Destruction order remains correct despite global var

## Done 

- [X] Forbid and detect creating new states in callback
- [X] Prevent getting the value from an observable, i.e. from the object SubscriptionObservable,  it is read-only
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

### Code 

- Use Result https://github.com/bitwizeshift/result
- Use fplus 
- More FP
- Use boost hana instead fo rolling our own bag of meta-programming tricks
- Look at https://github.com/xhawk18/promise-cpp

# Missing features

- Code is not thread-safe ! So using only mutually exclusive callback groups is mandatory. 
- Memory strategy is not implemented, but could be easily, simply add the arguments everywhere 
- LifecycleNodes are currently not supported. Please open an issue if this is a blocker for you.