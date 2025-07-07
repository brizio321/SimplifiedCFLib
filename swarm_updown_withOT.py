# Import other custom modules
import sys
sys.path.insert(0, 'Optitrack')

from CFLib.SimpleCF import SimpleCF
from CFLib.SimpleCFSwarm import SimpleCFSwarm

from utils.plot_fcn import plot_fcn
from OptitrackClient import OptitrackClient

# Import regular python packages 
import numpy as np
import time
from matplotlib import pyplot as plt

import cflib.crtp

def execute_up_down_relative(self):
    p0 = self.get_last_position()
    # Initial position
    self.go_to(p0[0], p0[1], z=0.3)
    time.sleep(2)

    self.go_to(p0[0], p0[1], z=0.45)
    time.sleep(2)

    self.go_to(p0[0], p0[1], z=0.60)
    time.sleep(2)

    self.go_to(p0[0], p0[1], z=1)
    time.sleep(2)

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    uris = ['radio://0/80/2M/E7E7E7E707', 
            'radio://0/80/2M/E7E7E7E706']

    frame_ids = [27, 28]

    start_time = time.time()

    # Create the OT client
    oc = OptitrackClient(start_time)

    for frame in frame_ids:    
        oc.track_object(frame)

    oc.identity_transformation()
    run_thread = oc.start()

    # Swarm
    simple_swarm = SimpleCFSwarm(uris)

    for idx in range(len(uris)):
        cf = simple_swarm._cfs[idx]
        frame = frame_ids[idx]
        oc.add_track_callback(frame, cf.send_external_pos)

    simple_swarm.enable_extpos_all()

    for cf in simple_swarm._cfs:
        cf.use_phlc = True

    simple_swarm.assign_execute_method(0, execute_up_down_relative)
    simple_swarm.assign_execute_method(1, execute_up_down_relative)

    simple_swarm.start_swarm()

    for t in simple_swarm.cfs_threads:
        t.join()

    # Close optitrack stream
    oc.stop()