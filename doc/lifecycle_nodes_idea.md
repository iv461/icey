# Idea how to support lifecycle nodes:

For the lifecycle nodes, we see patterns like these (https://github.com/ros-navigation/navigation2/blob/main/nav2_map_server/src/map_server/map_server.cpp): 

- Declare parameters in the node constructor 
- `on_configure`-phase: Get parameters, initialize publishers and subcribers, eventually call 

- The lifecycle node's concept violates fundamentally the RAII principle, more precisely the automatic destruction of objects. This means, the developer is forced to become a C programmer, writing a `free` function for every node. -> We can use here RAII if we wrap all the ROS-baggage in a single pointer.


# References: 




