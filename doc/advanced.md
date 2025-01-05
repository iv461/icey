# Advanced topics 

### Order of initialization 

In ICEY, the parameters are initialized first, then the subscribers, then the publishers, then the services, then the actions and 
only then the timers. 

The order of declaration is important to make certain correctness properties. 
The parameters are initialized first because their values may be needed for initializing the other things. 


TODO clarify order 