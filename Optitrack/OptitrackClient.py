from PythonNatNetSDK.NatNetClient import NatNetClient
from matplotlib import pyplot as plt
import numpy as np
import scipy.io as sio
import time
from threading import Event, Thread
import sys

class OptitrackClient:

    '''
    Important note
        The client_address and server_address default values are setted under the hypothesis that the pc working as client
        is connected (directly) via ethernet to the server, that is the machine running Motive.
        In particular, two static IP address are assigned to the machines connected via ethernet.
        The server, so the pc in the lab running Motive, has the address 192.168.100.2
        The client, the pc receiving data, has the address 192.168.100.1

        These settings can be modified, just be carefull when initializing an OptitrackClient object 
    '''
    
    def __init__(self, start_time, client_address="192.168.100.2", server_address="192.168.100.1"):
        # Initialize the NatNet client
        self._client = NatNetClient()

        # Set the IP addresses
        self._client.set_client_address(client_address)    # This machine (client PC)
        self._client.set_server_address(server_address)    # Motive machine (server)

        # Register the listener for rigid body data
        self._client.rigid_body_listener = self._receive_rigid_body_frame
        self._client.new_frame_listener = self._receive_frame_listener

        # Event to stop the streaming
        self._stop_streaming = Event()

        # List of tracked object
        self._tracked_objs = []

        # Dictionary containing, for each streaming_id, the list of tracked position
        self._tracked_pos = {}

        # Dictionary containing, for each streaming_id, the list of callbacks
        self._tracked_cbs = {}

        # Time
        self._start_time = start_time
        self._track_time = np.array([])

        # Coordinate Transformation
        self._coor_transformation_configured = False
        self._O_ot_in_uwb = None
        self._rot_ot2uwb = None

    def track_object(self, streaming_id):
        # Include a new object to the tracked list.
        # Objects are identified using the IDs setted in Motive for each rigid body.
        if streaming_id not in self._tracked_objs:
            self._tracked_objs.append(streaming_id)
            self._tracked_pos[streaming_id] = np.empty((3,0))

    def add_track_callback(self, streaming_id, callback):
        if streaming_id not in self._tracked_objs:
            self._tracked_objs.append(streaming_id)
            self._tracked_pos[streaming_id] = np.empty((3, 0))
        self._tracked_cbs[streaming_id] = callback

    def identity_transformation(self):
        self.load_configuration("Optitrack/config/default_config")

    def load_configuration(self, filename):
        config = sio.loadmat(filename)
        self._rot_ot2uwb = config['rot_ot2uwb']
        self._O_ot_in_uwb = config['O_ot_in_uwb']
        self._coor_transformation_configured = True

    def run(self):
        if not self._coor_transformation_configured:
            print("[OptitrackClient.run()] There is no coordinate transformation between OptiTrack and UWB.")
            print("If you don't want to configure the transformation, invoke the method identity_transformation() on the OptitrackClient object.")
            return
        # Start the asynchronous data thread
        self._client.run('d')
        self._stop_streaming.wait()
        print("Exiting.")
        self._client.shutdown()

    def start(self):
        run_thread = Thread(target=self.run)
        run_thread.start()
        return run_thread

    def stop(self):
        try:
            self._stop_streaming.set()
        except:
            pass

    def _receive_frame_listener(self, data):
        # At each new data packet, the receiving time is stored
        self._track_time = np.append(self._track_time, time.time()-self._start_time)

    def _receive_rigid_body_frame(self, new_id, position, rotation):
        # This function is invoked for each rigid body included in a new data packet
        if new_id not in self._tracked_objs:
            return
        new_pos = np.reshape(np.array(position), (3,))
        # print(self._tracked_pos[new_id])
        # print(new_pos)
        self._tracked_pos[new_id] = np.append(self._tracked_pos[new_id],
                np.reshape(new_pos, (3,1)), axis=1)

        # If the rigid body is associated to a callback, invoke it (after the coordinate is transformed)
        if new_id in self._tracked_cbs:
            # Transform coordinates from OT to UWB
            # p_uwb = O_ot_in_uwb + rot_ot2uwb @ p_ot
            p_uwb = self._O_ot_in_uwb.reshape((3,)) + self._rot_ot2uwb@new_pos

            # print(p_uwb)

            # Send the position information
            self._tracked_cbs[new_id](p_uwb.tolist())