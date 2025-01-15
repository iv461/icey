# The Data Flow Graph (DFG)

TODO rework this! Our model seems not well suited, are edges values or computations ? 
Every ros entity that can be assigned a callback group may require a different callback group 
to prevent deadlocks (premise 1). Given this, and given the proposition that callback group 
assignment is likely a graph coloring problem, and given the fact that graph coloring can either color edges or vertices, all ros entities must be either all vertices or all edges. 
Currently, the DFG model seems inconsistent/ contaains constradictions.

The data-flow graph constains as vertices:

- Subscribers (TF and regular ones)
- Timers 
- Publishers 
- Services
- (Service) Clients
- Computations

For some characterization, these vertices cannot have in-edges:

- Timers
- Subscribers

These **must** have in-edges: 

- Publishers 
- (Service) Clients 

These have inputs an outputs:

- Computations, meaning functions

We can build loops. These are detected at runtime and rejected. 


## Configuring the graph after parameters 

In ICEY, the DFG is static. It cannot be changed at runtime (inside callbacks for example) after it has been declared initially. There is one exception however: We can declare the graph after we have obtained the parameters from ROS. This offers more flexibilty, for example we can decide given the parameters on which topics to subscribe. 

For this, we simply need to call 