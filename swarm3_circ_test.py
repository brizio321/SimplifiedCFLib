import numpy as np
import time
from matplotlib import pyplot as plt

import threading

import cflib.crtp
from CFLib.SimpleCF import SimpleCF
from CFLib.SimpleCFSwarm import SimpleCFSwarm

# URI 3
def execute_unitary_circle_0(scf):
    offset = 0
    angles = np.arange(0, 2*np.pi, np.pi/20)

    # Initial position
    scf.go_to(np.cos(angles[0]+offset), np.sin(angles[0]+offset), z=0.3)
    time.sleep(5)
    for i in range(1, len(angles)):
        scf.go_to(np.cos(angles[i]+offset), np.sin(angles[i]+offset), z=0.3)
        time.sleep(0.5)

# URI 4
def execute_unitary_circle_1(scf):
    offset = 2*np.pi/3
    angles = np.arange(0, 2*np.pi, np.pi/20)

    # Initial position
    scf.go_to(np.cos(angles[0]+offset), np.sin(angles[0]+offset), z=0.5)
    time.sleep(5)
    for i in range(1, len(angles)):
        scf.go_to(np.cos(angles[i]+offset), np.sin(angles[i]+offset), z=0.5)
        time.sleep(0.5)

# URI 6
def execute_unitary_circle_2(scf):
    offset = 4*np.pi/3
    angles = np.arange(0, 2*np.pi, np.pi/20)

    # Initial position
    scf.go_to(np.cos(angles[0]+offset), np.sin(angles[0]+offset), z=0.7)
    time.sleep(5)
    for i in range(1, len(angles)):
        scf.go_to(np.cos(angles[i]+offset), np.sin(angles[i]+offset), z=0.7)
        time.sleep(0.5)

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    uris = ['radio://0/80/2M/E7E7E7E703',
            'radio://0/80/2M/E7E7E7E704', 
            'radio://0/80/2M/E7E7E7E706']

    simple_swarm = SimpleCFSwarm(uris)
    simple_swarm.assign_execute_method(0, execute_unitary_circle_0)
    simple_swarm.assign_execute_method(1, execute_unitary_circle_1)
    simple_swarm.assign_execute_method(2, execute_unitary_circle_2)

    simple_swarm.start_swarm()

    for t in simple_swarm.cfs_threads:
        t.join()

    for cf in simple_swarm._cfs:
        t = np.arange(0, np.size(cf._pos, 1))
        fig, ax = plt.subplots(1, 1, layout='constrained')
        ax.plot(t, cf._pos[0, :], t, cf._pos[1, :], t, cf._pos[2, :])
        ax.legend(['CF_x', 'CF_y', 'CF_z'])
    
    plt.show()