# The Data Flow Graph (DFG)

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
