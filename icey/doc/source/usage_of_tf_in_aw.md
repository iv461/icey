Searching for "lookupTransform" in AW yields these utility functions:

getTransform
getLatestTransform


Interesting case: 
mission_planner cpp 
must get the latest transform in a service call. 
The AW code does not wait, it looks up the latest in the buffer-> the TF Stream must be buffered !
-> But how do we wait for a transform in a service call ? 
How to await that the service is called ?

Another case: 

PlanningEvaluatorNode::updateCalculatorEgoPose uses swapped target and source frames ! as target frame 
-> we could just say we invert the transform after we got it ! 
 


But everywhere parameters are used as target frame, so we need to support that


pure_pursuit lateral waits until at least one pose is available, we could support this easily with 
co_await car_pose_sub.timeout(1s)

Results for "waitForTransform":

Not much, only:
-> We need to store the timeout inside the Stream, otherwise we cannot co_await with timeout !



Results for "canTransform": 

- Only before transform. Once before a call to `pcl_ros::transformPointCloud` in pointcloud_preprocessor/filter.cpp

- All other calls (there only four of them) are just before `lookupTransform`, i.e. completely unnecessary.