import numpy as np
import time

from threading import Barrier, Thread

import cflib.crtp
from CFLib.SimpleCF import SimpleCF

class SimpleCFSwarm():
    
    def __init__(self, uris):
        self._cfs = [SimpleCF(uri) for uri in uris]
        self.cfs_threads = []
        self._sync_barrier = None

    def enable_extpos(self, cf_idx):
        self._cfs.use_extpos = True

    def enable_extpos_all(self):
        for cf in self._cfs:
            cf.use_extpos = True
       
    def assign_execute_method(self, cf_idx, executed_method):
        self._cfs[cf_idx]._executed_function = executed_method

    def assign_all_empty(self):
        for cf in self._cfs:
            cf._executed_function = SimpleCFSwarm._empty

    def wait_all_ready(self):
        '''
        repeat = True
        while repeat:
            repeat = False
            for cf in self._cfs:
                if not cf.took_off():
                    repeat = True
                    break
            time.sleep(1)
        '''
        wait = True
        while wait:
            wait = self._sync_barrier.n_waiting == len(self._cfs)
            time.sleep(1)

    @staticmethod
    def _empty(cf):
        while True:
            pass

    def start_swarm(self):
        self._sync_barrier = Barrier(len(self._cfs))
        for cf in self._cfs:
            t = Thread( target=cf.start_drone, args=(True, self._sync_barrier) )
            self.cfs_threads.append( t )

        for t in self.cfs_threads:
            t.start()

    def kill_all(self):
        for cf in self._cfs:
            cf._close_commander()

        for t in self.cfs_threads:
            t._stop()