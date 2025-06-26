from Optitrack.PythonNatNetSDK.NatNetClient import NatNetClient
from matplotlib import pyplot as plt
import numpy as np
import time
from threading import Event, Thread

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

        # Dictionary containing, for each frame_id, the list of tracked position
        self._tracked_pos = {}

        # Dictionary containing, for each frame_id, the list of callbacks
        self._tracked_cbs = {}

        # Time
        self._start_time = start_time
        self._track_time = np.array([])

        # Coordinate Transformation
        self._uwbO2ot = np.array([0, 0.32, 0])
        # self._uwbO2ot = np.array([0, 0, 0])

    def set_uwbO_coordinates(self, offset):
        self._uwbO2ot = np.array(offset)

    def track_object(self, frame_id):
        # Include a new object to the tracked list.
        # Objects are identified using the IDs setted in Motive for each rigid body.
        if frame_id not in self._tracked_objs:
            self._tracked_objs.append(frame_id)
            self._tracked_pos[frame_id] = np.empty((3, 0))

    def add_track_callback(self, frame_id, callback):
        if frame_id not in self._tracked_objs:
            self._tracked_objs.append(frame_id)
            self._tracked_pos[frame_id] = np.empty((3, 0))
        self._tracked_cbs[frame_id] = callback

    def run(self):
        # Start the asynchronous data thread
        self._client.run('d')
        self._stop_streaming.wait()
        print("Exiting.")
        self._client.shutdown()

    def stop(self):
        self._stop_streaming.set()

    def _receive_frame_listener(self, data):
        # At each new data packet, the receiving time is stored
        self._track_time = np.append(self._track_time, time.time()-self._start_time)

    def _receive_rigid_body_frame(self, new_id, position, rotation):
        # This function is invoked for each rigid body included in a new data packet
        if new_id not in self._tracked_objs:
            return
        # new_pos = np.array( [[position[0]], [position[1]], [position[2]]] )
        new_pos = np.array( [[position[0]], [position[1] - 0.38], [position[2]]] )
        self._tracked_pos[new_id] = np.append(self._tracked_pos[new_id], new_pos, axis=1)

        # If the rigid body is associated to a callback, invoke it (after the coordinate is transformed)
        lpos = [position[i]-self._uwbO2ot[i] for i in range(len(position))]
        if new_id in self._tracked_cbs:
            self._tracked_cbs[new_id](lpos)