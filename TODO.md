# TODO 

## Robustness/ Preventing glitches: 

- We have a concept of a state that is owned by a single node only, but how can we enforce that the state is only published by a single node ? 
- Topo Sort ! Update is not correct otherwise.
- Forbid and detect creating new states in callback
- Forbid subscribing to the same topic that is published by the same node 
- Forbid and detect cycles in the DAG: React has a "cyclic dependency detected" thing
- Prevent getting the value from an observable, i.e. from the object SubscribedState,  it is read-only
- Disallow graph change after node is created. Detect it. Provide instead "Node enable" functionality if someone wants to disable a publisher/subscriber
- After we have a DAG, we may add topological sorting. But at first we need a useEffect hook that translates to a timer, otherwise we are not able to make the programm run w/o external events 
- Really think about memory leaks, they should not be possible !
- Make sure we are moving all lambdas so that the captured values are moved as well !
- Check reference counts on all shared ptr to understand whrere they are all incremented
- Allowing only readable and only writable nodes and publishing resulting nodes is enough to ensure DAG-propery. But is it expressive enough ?
- Allow movable ? Likely not
- Global variable: Check linkage issues regarding ODR: What happens if icey is included multiple times in the translation unit ? -> we likely need a context,

## Features 

### Must-have  for 0.1

- [] Unit-tests testing the node
- [] Doxygen parsable comments 

- [] Correct sequence: FIRST create all subscribers, then all publishers !!
- [] Full-featured parameters !
- [X] TF Buffer 
- [] .catch() promise for tF buffer would be useful to print the lookup error

- [] Class-based API since OOP is popular and required of components 

- [x] Services
- [] Publishing multiple outputs, i.e. single input, multiple output
- Private topic publishing -> maybe like python: if ~ in front of the topic name, then its private
- `onCleanup` function to be able to call cleanup of external libraries, e.g. ONNX runtime (that would normally be the Node's destructor)
- We need a way to stop the data-flow: Specialize Observable for std::optional<T>  and simply check if there is a value. This is needed to support stooping pipeline, i.e. received invalid values, or conditional publishing.

- `beforeInit()` and `afterInit` functions to be able to use the icey::node API for stuff that is not implemented. 
- Allow accessing the node: For logging and for all other objects
- Different Sync-policies, first implement latest
-  Synch-policies ?
* Every input may provide a frequency interpolator 
* A strategy chooses between "last message" or interpolate
* in case of interpolate, the queue must be long, in case of last message, the queue can be 1
- Pass timer to the callback to be able to implement one-off-timer (https://github.com/ros2/demos/blob/rolling/demo_nodes_cpp/src/timers/one_off_timer.cpp)

- [] Service client using timer and different callback groups: https://docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html

- [] For service client: then().except().finally() syntax to be notified when the service is not available (and nice promise syntax)

- Sometimes we want to use the values of parameters to initialize the other stuff, i.e. frames of tf (topics should not be needed). So we need to somehow first get the parameters and then update sub/pub 
- message_filter::Subscriber must be usable (or maybe replace with own filter)
- Support Image Transport subscriber -> otherwise people cannot subscribe to their compressed video feeds, port https://github.com/ros-perception/image_transport_tutorials

- Scout ROS 2 demos for missing features: https://github.com/ros2/demos
- Scout https://github.com/orgs/ros-perception/

- Look at https://github.com/ros-controls/control_toolbox
- Look at https://github.com/PickNikRobotics/RSL, maybe for monad, or for https://github.com/PickNikRobotics/RSL/blob/main/include/rsl/parameter_validators.hpp
- Look at https://control.ros.org/rolling/index.html how to write controllers

- Look at https://github.com/BehaviorTree/BehaviorTree.ROS2/tree/humble, maybe some ideas, but unlikely 

- Maybe create a sequence diagram with websequencediagrams.com

- [] Actions ? See https://github.com/BehaviorTree/BehaviorTree.ROS2/tree/humble

- There are other filter libraries like https://github.com/ros/filters/tree/ros2, but not really used and only for simplistic tasks

- Nice Gui: https://github.com/christophfroehlich/rqt_lifecycle_manager

- Other ROS2 support libraries. 

- Logging with FMT: https://github.com/facontidavide/ros2_logging_fmt?tab=readme-ov-file

### Nice-to-have

- Allow for shared-ptr messages for perf, i.e. not copying the whole message but just notifying it changes. For this we need to just strip the ptr when calling node->subscribe<Msg>
- Timer signal: like a signal, but a timer ! Basis for every signal-generator AND and the same time can be used as a timer. Commonly, one wants to publish something periodically.
- "Waiting currently on" verbose printing to see what's happening
- Prevent having to use an arrow -> only because everything needs to be reference-counted: Wrap the smart-ptr inside an object, i.e. use PIMPL OR even better, reference-track every thing internally in the staged list until the node is created, then attach everything to the node (!). And then simply return a weak reference (const T &) to the underlying objects
- A way to enable/disable the node 
- Maybe Simulink-style blocks, i.e. constant, step, function etc.
- tf2_ros Message filter: Just another filter: https://github.com/ros-perception/imu_pipeline/tree/ros2/imu_transformer
- Lifecycle Nodes ?
- Dynamic reconfigure without code-gen using boost hana (it can serialize structs) -> Look at PickNicks code-gen solution 

- How to implement RCLCPP_COMPONENTS_REGISTER_NODE ? We need a custom type that does all what what required

### Code 

- Use Result https://github.com/bitwizeshift/result
- Use fplus 
- More FP
- Use boost hana instead fo rolling our own bag of meta-programming tricks
- Look at https://github.com/xhawk18/promise-cpp

## Documentation 

- Document everything in a tutorial-style like modern JS frameworks (Svelte, Solid.js) do, otherwise adoptability will equal to zero 

## Bugs 

- [] Fix segfault on termination, cleanup properly

## Examples 

Add examples for common applications: 

- Kalman filter: Multiple inputs, interpolate 
- SLAM: Front-end and backend, running asynchronously
- Controller like PPC, gets position and velocity, etc. 

# Missing features

- Mutli-threading executor is not supported, everything is single-threaded. Therefore, not Callback-groups