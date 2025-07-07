import numpy as np
import time
from matplotlib import pyplot as plt

import cflib.crtp
from CFLib.SimpleCF import SimpleCF

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
    p0 = self.get_last_position()
    # Initial position
    print("Initial position")
    self.go_to(p0[0], p0[1], z=p0[2]+0.5)
    time.sleep(5)
    # Go up 
    print("Go up")
    self.go_to(p0[0], p0[1], z=p0[2]+0.8)
    time.sleep(5)
    # Go down
    print("Go down to initial position")
    self.go_to(p0[0], p0[1], z=p0[2]+0.5)
    time.sleep(5)

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()
    uri = 'radio://0/80/2M/E7E7E7E702'

    # Create a SimpleCF object providing uri
    cf = SimpleCF(uri)

    # Set the execute function
    cf._executed_function = execute_up_down_relative
    
    # Start the agent
    # Note: this is a blocking method. The flow of the code continues in cf.start_drone()
    #       and resumes from here just once that the execution is completed 
    cf.start_drone()

    # Various plots
    t = np.arange(0, np.size(cf._pos, 1))
    fig, ax = plt.subplots(1, 1, layout='constrained')
    ax.plot(t, cf._pos[0, :], t, cf._pos[1, :], t, cf._pos[2, :])
    ax.legend(['CF_x', 'CF_y', 'CF_z'])
    plt.show()
