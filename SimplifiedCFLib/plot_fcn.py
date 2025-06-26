from matplotlib import pyplot as plt
from scipy import interpolate
import numpy as np

def plot_fcn(oc, cf, frame_id):
    fig, ax = plt.subplots(1, 1, layout='constrained')
    ax.plot(oc._track_time, oc._tracked_pos[frame_id][0, :], 
            oc._track_time, oc._tracked_pos[frame_id][1, :], 
            oc._track_time, oc._tracked_pos[frame_id][2, :])
    ax.legend(['CF_x', 'CF_y', 'CF_z'])
    ax.set_title('OptiTrack Trajectory', fontweight ="bold")

    fig, ax = plt.subplots(1, 1, layout='constrained')
    ax.plot(cf._pos_time, cf._pos[0, :], 
            cf._pos_time, cf._pos[1, :], 
            cf._pos_time, cf._pos[2, :])
    ax.legend(['CF_x', 'CF_y', 'CF_z'])
    ax.set_title('CF Estimated Trajectory', fontweight ="bold")

    fig, ax = plt.subplots(3, 1, layout='constrained')
    ax[0].plot(oc._track_time, oc._tracked_pos[frame_id][0, :],
                cf._pos_time, cf._pos[0, :] )
    ax[0].legend(['OT_x', 'CF_x'])
    ax[0].set_title('OT vs CF Estimated Trajectory', fontweight ="bold")

    ax[1].plot(oc._track_time, oc._tracked_pos[frame_id][1, :],
                cf._pos_time, cf._pos[1, :] )
    ax[1].legend(['OT_y', 'CF_y'])

    ax[2].plot(oc._track_time, oc._tracked_pos[frame_id][2, :],
                cf._pos_time, cf._pos[2, :] )
    ax[2].legend(['OT_z', 'CF_z'])

    # Error plot
    f_x = interpolate.interp1d(cf._pos_time, cf._pos[0, :])
    f_y = interpolate.interp1d(cf._pos_time, cf._pos[1, :])
    f_z = interpolate.interp1d(cf._pos_time, cf._pos[2, :])

    idx_t0 = np.argmax( oc._track_time - cf._pos_time[0] > 0 )
    idx_tf = np.argmax( oc._track_time - cf._pos_time[-1] > 0 )

    fig, ax = plt.subplots(3, 1, layout='constrained')
    ax[0].plot(oc._track_time[idx_t0:idx_tf], oc._tracked_pos[frame_id][0, idx_t0:idx_tf]-f_x(oc._track_time[idx_t0:idx_tf]) )
    ax[0].legend(['OT_x - CF_x'])
    ax[0].set_title('Error', fontweight ="bold")

    ax[1].plot(oc._track_time[idx_t0:idx_tf], oc._tracked_pos[frame_id][1, idx_t0:idx_tf]-f_y(oc._track_time[idx_t0:idx_tf]) )
    ax[1].legend(['OT_y - CF_y'])

    ax[2].plot(oc._track_time[idx_t0:idx_tf], oc._tracked_pos[frame_id][2, idx_t0:idx_tf]-f_z(oc._track_time[idx_t0:idx_tf]) )
    ax[2].legend(['OT_z - CF_z'])
