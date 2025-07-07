# Import other custom modules
import sys
sys.path.insert(0, '../')
from CFLib.SimpleCF import SimpleCF
from Optitrack.OptitrackClient import OptitrackClient

# Import regular python packages 
from scipy import interpolate
import numpy as np
from scipy.io import savemat


def export_drone_position(cf: SimpleCF, filename: str):
    data = {
            "uri": cf._uri, 
            "time": cf._pos_time, 
            "pos": cf._pos.T
            }
    savemat(filename, data, appendmat=True)

def export_drone_ot_position(cf: SimpleCF, frame_id: int, oc: OptitrackClient, filename: str):
    idx_t0, idx_tf, fitted_cf_est = fit_data(cf, oc)
    data = {
            "uri": cf._uri, 
            "time": oc._track_time[idx_t0:idx_tf], 
            "cf_est_pos": fitted_cf_est.T,
            "ot_est_pos": oc._tracked_pos[frame_id][:, idx_t0:idx_tf]
            }
    savemat(filename, data)

def fit_data(cf: SimpleCF, oc: OptitrackClient):
    f_x = interpolate.interp1d(cf._pos_time, cf._pos[0, :])
    f_y = interpolate.interp1d(cf._pos_time, cf._pos[1, :])
    f_z = interpolate.interp1d(cf._pos_time, cf._pos[2, :])

    idx_t0 = np.argmax( oc._track_time - cf._pos_time[0] > 0 )
    idx_tf = np.argmax( oc._track_time - cf._pos_time[-1] > 0 )

    tmp = np.vstack(
        (   f_x(oc._track_time[idx_t0:idx_tf]), 
            f_y(oc._track_time[idx_t0:idx_tf]), 
            f_z(oc._track_time[idx_t0:idx_tf])  
        )
    )

    return (idx_t0, idx_tf, tmp.T)