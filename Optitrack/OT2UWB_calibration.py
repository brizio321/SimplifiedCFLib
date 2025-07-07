import numpy as np
import time
import sys

from matplotlib import pyplot as plt

from scipy import interpolate
from scipy.spatial.transform import Rotation as R
from scipy.io import savemat

from threading import Thread

import cflib.crtp

from OptitrackClient import OptitrackClient

# setting path
sys.path.append('../SimplifiedCFLib')
from CFLib.SimpleCF import SimpleCF

'''
Sequence of movements perfromed to collect UWB position estimation and OT position estimation.
UWB are then oversampled to fit OT measurements, so a MS estimation of the transformation between 
two reference frames is computed.
'''

def move(self):
    # Get the initial position, it is necessary to keep x and y coordinates in time.
    p0 = self.get_last_position()

    print(p0)

    # Move up and down
    self.go_to(p0[0], p0[1], z=0.3)
    time.sleep(3)
    self.go_to(p0[0], p0[1], z=0.7)
    time.sleep(3)
    self.go_to(p0[0], p0[1], z=0.4)
    time.sleep(3)

    # Move along x axis
    self.go_to(p0[0]+0.4, p0[1], z=0.60)
    time.sleep(3)
    self.go_to(p0[0]-0.4, p0[1], z=0.60)
    time.sleep(3)
    self.go_to(p0[0], p0[1], z=0.60)
    time.sleep(3)

    # Move along y axis
    self.go_to(p0[0], p0[1]+0.4, z=0.60)
    time.sleep(3)
    self.go_to(p0[0], p0[1]-0.4, z=0.60)
    time.sleep(3)
    self.go_to(p0[0], p0[1], z=0.60)
    time.sleep(3)

if __name__ == '__main__':
    # Initialize the low-level drivers, necessary to control an agent.
    cflib.crtp.init_drivers()

    # Define the URI identifying the agent
    uri = 'radio://0/80/2M/E7E7E7E706'

    # Store the initial time. 
    # This is necessary just to compare the position estimation given by UWB and OT. 
    start_time = time.time()

    # Create a SimpleCF object providing uri and start_time
    cf = SimpleCF(uri, start_time)
    cf.use_phlc = False

    # Set the execute function
    cf._executed_function = move
    
    # Create the OT client
    oc = OptitrackClient(start_time)

    # Add to the client ALL the object that you want to track.
    # If more objects must be tracked, invoke
    #   oc.track_object(id)
    # specifying all the others ids    
    frame_id = 27
    oc.track_object(frame_id)

    # Start the OC client in a separate thread. This is necessary
    # to run in parallel the control of the agent 
    # (I'll simplify this in the next code release)
    oc.identity_transformation()
    run_thread = Thread(target=oc.run)
    run_thread.start()

    # Enable external position reference
    cf.use_extpos = False

    # Add the drone to the list of objects that must be tracked
    # uri and frame_id refere to the same agent, but in different context:
    # it may be helpful to define the frame_id in Motive equals to the uri,
    # or at least the last digits of the uri
    oc.add_track_callback(frame_id, cf.send_external_pos)

    # Start the agent
    # Note: this is a blocking method. The flow of the code continues in cf.start_drone()
    #       and resumes from here just once that the execution is completed 
    cf.start_drone()

    # Oversample UWB data to fit OT observations
    f_x = interpolate.interp1d(cf._pos_time, cf._pos[0, :])
    f_y = interpolate.interp1d(cf._pos_time, cf._pos[1, :])
    f_z = interpolate.interp1d(cf._pos_time, cf._pos[2, :])

    # Time interval where both signals are collected
    idx_t0 = np.argmax( oc._track_time - cf._pos_time[0] > 0 )
    idx_tf = np.argmax( oc._track_time - cf._pos_time[-1] > 0 )

    # Compute the rotation transform
    # Rotate OT frame in UWB frame
    tmp = np.vstack(( f_x(oc._track_time[idx_t0:idx_tf]), f_y(oc._track_time[idx_t0:idx_tf]), f_z(oc._track_time[idx_t0:idx_tf]) ))    
    rot_ot2uwb, rssd = R.align_vectors(tmp.T, oc._tracked_pos[frame_id][:, idx_t0:idx_tf].T)

    # p_uwb = O_ot_in_uwb + rot_ot2uwb @ p_ot
    O_ot_in_uwb = cf._pos[:, idx_t0] - rot_ot2uwb.as_matrix()@oc._tracked_pos[frame_id][:, idx_t0]

    # Store the configuration
    filename = "ot_uwb_config"
    transformation = {"rot_ot2uwb": rot_ot2uwb.as_matrix(), "O_ot_in_uwb": O_ot_in_uwb}

    savemat(filename, transformation)

    # Close optitrack stream
    oc.stop()