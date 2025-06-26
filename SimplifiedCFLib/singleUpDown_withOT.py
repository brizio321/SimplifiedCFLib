import numpy as np
import time
from matplotlib import pyplot as plt

from plot_fcn import plot_fcn

from scipy import interpolate

from threading import Thread

import cflib.crtp
from CFLib.SimpleCF import SimpleCF

from Optitrack.OptitrackClient import OptitrackClient

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

def execute_up_down_relative(self):
    '''
    This is the method describing the action computed by the controlled agent.
    It has to be defined with the sign
        def method(self):
            .
            .   
            .
    in order to be compatible with the SimpleCF class.

    In this method it is possible to send position references to the agent
    by invoking self.go_to(x, y, z).

    In details, the action designed here let the drone move increasing 
    its z coordinate, while keeping x and y coordinates constant.
    '''

    # Get the initial position, it is necessary to keep x and y coordinates in time.
    p0 = self.get_last_position()

    self.go_to(p0[0, 0], p0[1, 0], z=0.3)
    time.sleep(1)

    self.go_to(p0[0, 0], p0[1, 0], z=0.45)
    time.sleep(1)

    self.go_to(p0[0, 0], p0[1, 0], z=0.60)
    time.sleep(1)

    self.go_to(p0[0, 0], p0[1, 0], z=0.75)
    time.sleep(1)

if __name__ == '__main__':
    # Initialize the low-level drivers, necessary to control an agent.
    cflib.crtp.init_drivers()

    # Define the URI identifying the agent
    uri = 'radio://0/80/2M/E7E7E7E704'

    # Store the initial time. 
    # This is necessary just to compare the position estimation given by UWB and OT. 
    start_time = time.time()

    # Create a SimpleCF object providing uri and start_time
    cf = SimpleCF(uri, start_time)
    cf.use_phlc = False

    # Set the execute function
    cf._executed_function = execute_up_down_relative
    
    # Create the OT client
    oc = OptitrackClient(start_time)

    # Add to the client ALL the object that you want to track.
    # If more objects must be tracked, invoke
    #   oc.track_object(id)
    # specifying all the others ids    
    frame_id = 29
    oc.track_object(frame_id)

    # Start the OC client in a separate thread. This is necessary
    # to run in parallel the control of the agent 
    # (I'll simplify this in the next code release)
    run_thread = Thread(target=oc.run)
    run_thread.start()

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

    # Close optitrack stream (idk why, but it has to be closed managing an exception)
    try:
        oc.stop()
    except:
        pass

    # Various plots
    plot_fcn(oc, cf, frame_id)

    plt.show()
