# Idea how to support lifecycle nodes:

For the lifecycle nodes, we see patterns like these (https://github.com/ros-navigation/navigation2/blob/main/nav2_map_server/src/map_server/map_server.cpp): 

- Declare parameters in the node constructor 
- `on_configure`-phase: Get parameters, initialize publishers and subcribers, eventually call 



# References: 




