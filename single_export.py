# Import other custom modules
import sys
sys.path.insert(0, 'Optitrack')

from CFLib.SimpleCF import SimpleCF
from utils.plot_fcn import plot_fcn
from OptitrackClient import OptitrackClient

from utils.export_methods import export_drone_ot_position

# Import regular python packages 
import numpy as np
import time
from matplotlib import pyplot as plt

import cflib.crtp

'''
Example of how to use the SimpleCF class to drive a drone integrating the position
estimation provided by Optitrack.

Optitrack data can be used in two different ways:
    1) Used as ground truth; in this way it is possible to compare UWB estimation with
        Optitrack estimation.
    2) It can be forwarded to the crazyflie and included in information used to estimate
        the position of the agent.

In this example, only one agent is controlled.
Two info are necessary to complete correctly a run of this example: the URI of the CF and the 
FrameID used in Motive to identify the rigid body.
Here we used the URI radio://0/80/2M/E7E7E7E702 and, from Motive, we created a rigid body assigning
the FrameID 28.

In case more than one agent is controlled simultaneously, pay attention to match URIs and FrameIDs to avoid
unpredictable results.

The correct way to associate an URI to a FrameID is discussed along this example.
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
    uri = 'radio://0/80/2M/E7E7E7E701'

    # Store the initial time. 
    # This is necessary just to compare the position estimation given by UWB and OT. 
    start_time = time.time()

    # Create a SimpleCF object providing uri and start_time
    cf = SimpleCF(uri, start_time)
    cf.use_phlc = True

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

    # Start the OC client
    oc.identity_transformation()
    run_thread = oc.start()

    # Enable external position reference
    cf.use_extpos = True

    # Add the drone to the list of objects that must be tracked
    # uri and frame_id refere to the same agent, but in different context:
    # it may be helpful to define the frame_id in Motive equals to the uri,
    # or at least the last digits of the uri
    oc.add_track_callback(frame_id, cf.send_external_pos)

    # Start the agent
    # Note: this is a blocking method. The flow of the code continues in cf.start_drone()
    #       and resumes from here just once that the execution is completed 
    cf.start_drone()

    # Close optitrack stream
    oc.stop()

    # Export
    # filename = os.path.abspath(os.getcwd()) + "/drone_" + uri + ".mat"
    filename = "drone_" + uri[-1] + ".mat"
    export_drone_ot_position(cf, frame_id, oc, filename)

    # Various plots
    plot_fcn(oc, cf, frame_id)

    plt.show()