from functools import partial 

import rclpy

from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

g_node = None 

class NodeWrap(Node):
    def __init__(self, name):
        super().__init__(name)
        self.my_subs = {}
        self.my_pubs = {}

def launch_node():
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(g_node)
    executor.spin()

class Observable:
    def __init__(self):
        self.notify_list = []

    def on_change(self, msg):
        for cb in self.notify_list:
            cb(msg)

    def notify(self, cb):
        self.notify_list.append(cb)

class ReadableState(Observable): 

    def __init__(self, name: str, state_content: type):
        super(Observable).__init__()

class WritableState(Observable):

    def __init__(self):
        super(Observable).__init__()

    def publish(self, topic_name, **kwargs):
        self.publishers.append(Publisher(topic_name))

class ProxiedMsg(MutableState):

    def __getitem__(self, name):

    def __setitem__(self, name, value):
        ... 


class SubscribedState(ReadableState):
    # State that is stored somewhere else in a node
    def __init__(self, name: str, state_content: type):
        super(ReadableState).__init__()

class PublishedState(WritableState):
    # State that is public, i.e. published
    # The max frequency is a limit of the maximum frequency this message will be published at. 
    # If unlimited, then this potentially may cause 100% CPU usage due to positive feedback with other nodes. 
    def __init__(self, name: str, state_content: type, max_frequency: None | float):
        super(ReadableState).__init__()

class PrivateState(MutableState): 
    # State that is not published, otherwise same 
    # Can be enabled to be a published state at any time, usefull for debugging.
    def __init__(self, name: str, state_content: type):

class spawn_node:
    def __enter__(self):
        # Start recording pub/sub
    def __exit__(self):
        # spawn_node 
        # clear all added 

class RTFunction:
    # A wrapper around a function that the user knows is referentially transparent (RT), meaning with same input 
    # it always has the same output. The user has the responsibility to ensures this. 
    # This implies for example that the algorithm is deterministic. Another criterion is that the function is pure, i.e. it does not access 
    # global variables (no static variables as well) etc.
    def __init__(self, cb):
        self.cb = cb 
    def __call__(self, **kwargs): self.cb(kwargs)

def spawn(node_name: None | str):
    global g_node
    launch_node(node_name)
    g_node = None # TODO correct ? 

def compute_based_on(F, *args):
    computation = partial(F, *args) # 
    result = WritableState()

    def continued():
        new_state = computation()
        result.on_change(new_state)

    for arg in *args:
        arg.notify_list.append(continued)

    return result

def example1():

    # allow for plumbing: 
    current_pose = SubscribedState("car_pose", states.TF) # subscribes automatically on /tf 

    #target_vel = 100 if (current_pose.x - 100) < 100 else 0
    current_velocity = PrivateState("current_velocity", float)

    param_max_vel = ParameterState("v_max", float) # write using param mechanism of ros 

    def compute_vel(pose):

    derived_state = compute_based_on(compute_vel, current_pose) # Read-only derived state

    derived_state.publish("velocity")

    spawn()




