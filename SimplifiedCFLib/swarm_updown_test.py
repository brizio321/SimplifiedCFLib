import numpy as np
import time
from matplotlib import pyplot as plt

import threading

import cflib.crtp
from CFLib.SimpleCF import SimpleCF
from CFLib.SimpleCFSwarm import SimpleCFSwarm

def execute_up_down_relative(self):
    p0 = self.get_last_position()
    # Initial position
    print("Initial position")
    self.go_to(p0[0][0], p0[1][0], z=p0[2][0]+0.5)
    time.sleep(2)
    # Go up 
    print("Go up")
    self.go_to(p0[0][0], p0[1][0], z=p0[2][0]+0.8)
    time.sleep(2)
    # Go down
    print("Go down to initial position")
    self.go_to(p0[0][0], p0[1][0], z=p0[2][0]+0.5)
    time.sleep(2)

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    uris = ['radio://0/80/2M/E7E7E7E703', 
            'radio://0/80/2M/E7E7E7E702']

    simple_swarm = SimpleCFSwarm(uris)
    simple_swarm.assign_execute_method(0, execute_up_down_relative)
    simple_swarm.assign_execute_method(1, execute_up_down_relative)

    simple_swarm.start_swarm()