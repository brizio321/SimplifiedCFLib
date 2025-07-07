# General purpose import
import time
import numpy as np
from matplotlib import pyplot as plt

from threading import Event, Thread, BrokenBarrierError

# Crazyflie import
import cflib.crtp

# The Crazyflie class is used to easily connect/send/receive data 
# from a Crazyflie.
from cflib.crazyflie import Crazyflie

# The synCrazyflie class is a wrapper around the “normal” Crazyflie class. 
# It handles the asynchronous nature of the Crazyflie API and turns it 
# into blocking function.
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# LogConfig class is a representation of one log configuration that enables
# logging from the Crazyflie
from cflib.crazyflie.log import LogConfig

from cflib.utils.callbacks import Caller

# Object used to send external position references to a drone
from cflib.crazyflie.extpos import Extpos

from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.positioning.motion_commander import MotionCommander

# Default parameters, you can change them
DEFAULT_VELOCITY = 0.25
DEFAULT_HEIGHT = 0.3
DEFAULT_LANDING_HEIGHT = 0.1

class SimpleCF:

    '''
    SimpleCF class has the aim to provide a simplified software interface that 
    can be used to control a Crazyflie drone.

    It tries to:
        - Hide "unnecessary" options
        - Group and automate some lines of code necessary to drive a Crazyflie 
    '''

    def __init__(self, uri, start_time=None, Ts=100):
        '''
        Here is a brief description of the parameters required to create a SimpleCF object.

        uri:    The URI used to identify the drone

        Ts:     Sampling time used to log data from the drone

        start_time: Initial time of the execution
        '''

        # Store the initial time of the execution
        if start_time is None:
            self._start_time = time.time()
        else:
            self._start_time = start_time

        # Store uri and log sampling time
        self._uri = uri
        self._ts = Ts

        # Status variables
        # Check if the drone is connected to the pc
        self.is_offline = True

        # Log variables and log initial configuration
        self._first_log_arrived = False
        self._log_conf = LogConfig(name="cf_log_conf", period_in_ms=Ts)
        self._default_log_config()

        # Function containing the action that the agent has to perform.
        # Initially it is set just to a time.wait(5)
        self._executed_function = self._execute

        # Flag necessary to distinguish single mode and swarm mode
        self._swarm_mode = False

        # Enable external position source
        self.use_extpos = False
        self.extpos_recevier = None
        
        # Position log data
        # Data estimated and logged by the drone
        self._pos = np.empty((3, 0))
        # Time associated to position logs
        self._pos_time = np.array([])

        # Control data: positions setpoint vector
        self._pos_set_point = None

        # Two commanders are supported: PositionHlCommander and Commander.
        # The two commanders are exclusive each other: just one for instance of the class
        #   can be used.
        self._pos_hl_commander = None
        self._motion_commander = None
        self._commander = None

        # Check which commander object is used by the instance
        self.use_phlc = True

        # Commander needs a support thread to be used. This thread is in charge of continously
        #   send the position setpoint to the agent. Moreover, an Event is used to communicate
        #   with the thread and stop it when the execution is completed.
        self._commander_support_thread_stop = Event()
        self._commander_support_thread = Thread( target=self._send_position_setpoint )

    def connection_established(self, *args):
        print("[simpleCF.connection_established()] Connected to the drone ", self._uri)
        self.is_offline = False

    def connection_lost(self, *args):
        print("[SimpleCF.connection_lost()] No longer connected to the drone ", self._uri)
        self.is_offline = True

    def add_log_variable(self, name, fetch_as):
        self._log_conf.add_variable(name, fetch_as)

    def _default_log_config(self):
        # Add the components of the state logged by default
        self._log_conf.add_variable('kalman.stateX', 'float')
        self._log_conf.add_variable('kalman.stateY', 'float')
        self._log_conf.add_variable('kalman.stateZ', 'float')        

    '''
    Start and stop methods
    '''
    def start_drone(self, swarm_mode=False, barrier=None):
        # Open SyncCrazyflie context
        self._swarm_mode = swarm_mode
        with SyncCrazyflie(self._uri, cf = Crazyflie(rw_cache='./cache')) as scf:
            # Store che SyncCrazyflie object
            self._scf = scf

            # Modify callbacks for connection established and lost
            conn_cb = Caller()
            conn_cb.add_callback(self.connection_established)
            scf.cf.fully_connected = conn_cb
            
            lost_conn_cb = Caller()
            lost_conn_cb.add_callback(self.connection_lost)
            scf.cf.connection_lost = lost_conn_cb

            # Reset KF and set the drone in Position Control mode
            scf.cf.param.set_value('stabilizer.controller', '3')
            scf.cf.param.set_value('kalman.resetEstimation', '1')
            time.sleep(2)

            # Load the log configuration
            scf.cf.log.add_config(self._log_conf)
            self._log_conf.data_received_cb.add_callback(
                lambda _timestamp, data, _logconf: self._async_log_cb(data)
            )

            # Start state log
            self._log_conf.start()

            # Wait for the first log in order to know the position
            while not self._first_log_arrived:
                pass

            # Enable external position source
            if self.use_extpos:
                self._enable_ext_pos()

            # Do stuff
            if swarm_mode:
                self._execute_swarm_wrapper(barrier)
            else:
                self._execute_wrapper()

            # Stop state log
            self._log_conf.stop()
        print("[SimpleCF.start_drone()] Operation completed.")

    def stop_drone(self):
        # Let the drone land
        if self.use_phlc:
            self._close_phlcommander()
        else:
            self._close_commander()

    '''
    Check take off methods
    '''
    def wait_for_take_off(self):
        while not self.took_off():
            time.sleep(0.2)

    def took_off(self):
        # Check if the agent is flying
        if self._pos_hl_commander is None and self._motion_commander is None:
            return False
        if self.use_phlc:
            return self._pos_hl_commander._is_flying
        else:
            return self._motion_commander._is_flying

    '''
    Execution wrapper methods
    '''
    def _execute_wrapper(self):
        '''
        This function, as the name suggets, wrap the execution of the action that 
        the agent has to complete.

        Here initialization are performed before the drone take off.
        Then, the action is performed.
        Finally, the drone land safetly and all the objects used to control the drone
        are properly closed/managed. 
        '''
        if not self.use_phlc:
            self._init_commander()
            self._executed_function(self)
            self._close_commander()
        else:
            self._init_phlcommander()
            self._executed_function(self)
            self._close_phlcommander()
        print("[SimpleCF._execute_wrapper()] End of execution.")

    def _execute_swarm_wrapper(self, barrier):
        '''
        This function, as the name suggets, wrap the execution of the action that 
        the agent has to complete IN A SWARM FORMATION.

        This include an initial waiting phase until all the agents take off.
        The barrier wait has exactly this goal.
        '''
        if not self.use_phlc:
            self._init_commander()
            try:
                print("Drone ", self._uri, " waiting for the others...")
                barrier.wait(timeout=15)
                self._executed_function(self)
            except BrokenBarrierError:
                print("[SimpleCF._execute_swarm_wrapper()] Barrier time out reached. Landing...")
            except Exception as err:
                print("[SimpleCF._execute_swarm_wrapper()] An error occurred during initialization: ")
                print(err)
                print("Landing...")
            self._close_commander()
        else:
            while not self._first_log_arrived:
                time.sleep(1)
            self._init_phlcommander()
            # Wait for all the drones to take off
            try:
                print("Drone ", self._uri, " waiting for the others...")
                barrier.wait(timeout=15)
                self._executed_function(self)
            except BrokenBarrierError:
                print("[SimpleCF._execute_swarm_wrapper()] Barrier time out reached. Landing...")
            except Exception as err:
                print("[SimpleCF._execute_swarm_wrapper()] An error occurred during initialization: ")
                print(err)
                print("Landing...")
            self._close_phlcommander()
                
        print("[SimpleCF._execute_wrapper()] End of execution.")

    '''
    Commanders initialization methods 
    '''
    def _init_commander(self):
        mc = MotionCommander(self._scf, default_height=DEFAULT_HEIGHT)
        mc.take_off()
        time.sleep(3)
        self._motion_commander = mc
        self._commander = self._scf.cf.commander
        self.wait_for_take_off()
        self._commander_support_thread.start()
        while not self._commander_support_thread.is_alive():
            time.sleep(0.2)

    def _init_phlcommander(self):
        while not self._first_log_arrived:
            time.sleep(0.2)
        p0 = self.get_last_position() 
        phlc = PositionHlCommander(self._scf, x=p0[0], y=p0[1], z=0, 
                default_velocity=DEFAULT_VELOCITY, default_height=DEFAULT_HEIGHT, 
                controller = PositionHlCommander.CONTROLLER_PID, default_landing_height = DEFAULT_LANDING_HEIGHT)
        phlc.take_off()
        time.sleep(3)
        self._pos_hl_commander = phlc
        self._commander = self._scf.cf.commander
        print("Wait take off...")
        self.wait_for_take_off()

    def _close_commander(self):
        self._commander_support_thread_stop.set()
        self._motion_commander.land()
        # time.sleep(3)

    def _close_phlcommander(self):
        self._pos_hl_commander.land()
        # time.sleep(3)

    '''
    '''
    def _execute(self, this):
        print("Execute: sleep(5)")
        time.sleep(5)

    '''
    External position methods
    '''
    def _enable_ext_pos(self):
        self.extpos_recevier = Extpos(self._scf.cf)

    def send_external_pos(self, pos):
        if self.extpos_recevier is None:
            return
        x = pos[0]
        y = pos[1]
        z = pos[2]
        self.extpos_recevier.send_extpos(x, y, z)

    '''
    Control methods
    '''
    def go_to(self, x, y, z):
        if self._pos_hl_commander == None and self._commander == None:
            print("[SimpleCF.go_to()] No commander object available.")
            return

        # Update the setpoint, so the _commander_support_thread can access it
        self._pos_set_point = np.array([x, y, z])

        # If PositionHlCommander is used, send the setpoint via object
        if self.use_phlc:
            self._pos_hl_commander.go_to(x, y, z)

    def _send_position_setpoint(self):
        # Method used to init the _commander_support_thread object.
        # It sends continously the position reference to the drone via _commander object instance.
        print("SEND_POSITION_THREAD STARTED")
        while not self._commander_support_thread_stop.is_set():
            if self._pos_set_point is None:
                # print("SEND_POS_THREAD No position")
                continue
            # print("SEND_POS_THREAD")
            x = self._pos_set_point[0]
            y = self._pos_set_point[1]
            z = self._pos_set_point[2]
            self._commander.send_position_setpoint(x, y, z, 0)
        print("SEND_POSITION_THREAD STOPPPED")

    '''
    Log and getter methods
    '''
    def _async_log_cb(self, data):
        self._default_log_cb(data)
        # Add eventual logic to manage custom logs

    def _default_log_cb(self, data):
        # Reconstruct position vector
        pos = np.array([[data['kalman.stateX']],
                        [data['kalman.stateY']], 
                        [data['kalman.stateZ']]]
                    )

        # Store current position
        self._pos_time = np.append(self._pos_time, time.time()-self._start_time)
        self._pos = np.append(self._pos, pos, axis=1)
        if not self._first_log_arrived:
            self._first_log_arrived = True

    def get_last_position(self):
        # Return the last logged position
        return np.reshape(self._pos[:, np.size(self._pos, 1)-1], (3,))
        
if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()
    uri = 'radio://0/80/2M/E7E7E7E704'

    cf = SimpleCF(uri)
    cf.start_drone()

    t = np.arange(0, np.size(cf._pos, 1))
    fig, ax = plt.subplots(1, 1, layout='constrained')
    ax.plot(t, cf._pos[0, :], t, cf._pos[1, :], t, cf._pos[2, :])
    ax.legend(['CF_x', 'CF_y', 'CF_z'])
    plt.show()