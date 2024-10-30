import pickle as pkl
import argparse
import numpy as np
import sys
from scipy.signal import savgol_filter

from force_dmp import ForceDMP

from movement_primitives.dmp import CartesianDMP, DMP

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--demo_name', '-dn')
    parser.add_argument('--pose_bfs', '-pbf', type=int, default=1000)
    parser.add_argument('--force_bfs', '-fbf', type=int, default=1000)
    parser.add_argument('--save_name', '-sn')
    args = parser.parse_args()
    
    pose_traj = pkl.load(open('hybrid_control/saved_demos/' + args.demo_name + '_pose_traj.pkl', 'rb'))
    pose_demo = [pose.position for pose in pose_traj]
    pose_demo = np.array(pose_demo)
    T = pose_demo.shape[0] / 100
    dt = 0.01
    
    force_demo = pkl.load(open('hybrid_control/saved_demos/' + args.demo_name + '_target_forces.pkl', 'rb'))
    force_demo = np.array(force_demo)

    force_demo[:,0] = savgol_filter(force_demo[:,0], 51, 3)
    force_demo[:,1] = savgol_filter(force_demo[:,1], 51, 3)
    force_demo[:,2] = savgol_filter(force_demo[:,2], 51, 3)
    force_demo[:,3] = savgol_filter(force_demo[:,3], 51, 3)
    force_demo[:,4] = savgol_filter(force_demo[:,4], 51, 3)
    force_demo[:,5] = savgol_filter(force_demo[:,5], 51, 3)

    tau = 1.0

    fdmp_x = ForceDMP(T,dt,n_bfs=args.force_bfs)
    fdmp_x.fit(force_demo[:,0], tau=tau)
    fdmp_y = ForceDMP(T,dt,n_bfs=args.force_bfs)
    fdmp_y.fit(force_demo[:,1], tau=tau)
    fdmp_z = ForceDMP(T,dt,n_bfs=args.force_bfs)
    fdmp_z.fit(force_demo[:,2], tau=tau)
    fdmp_ro = ForceDMP(T,dt,n_bfs=args.force_bfs)
    fdmp_ro.fit(force_demo[:,3], tau=tau)
    fdmp_pi = ForceDMP(T,dt,n_bfs=args.force_bfs)
    fdmp_pi.fit(force_demo[:,4], tau=tau)
    fdmp_ya = ForceDMP(T,dt,n_bfs=args.force_bfs)
    fdmp_ya.fit(force_demo[:,5], tau=tau)

    fdmp = {}
    fdmp['x'] = fdmp_x
    fdmp['y'] = fdmp_y
    fdmp['z'] = fdmp_z
    fdmp['ro'] = fdmp_ro
    fdmp['pi'] = fdmp_pi
    fdmp['ya'] = fdmp_ya

    execution_time = pose_demo.shape[0] / 100
    Y = np.array(pose_demo.copy())
    Y_f = np.array(force_demo.copy())
    T = np.linspace(0, execution_time, Y.shape[0])
    quat_demo = [pose.quaternion for pose in pose_traj]
    quat_demo = np.array(quat_demo)
    Y = np.concatenate((Y, quat_demo), axis=1)
    dmp = CartesianDMP(execution_time=execution_time, dt=dt, n_weights_per_dim=args.pose_bfs)
    alpha_y = 25.0
    beta_y = (25.0 / 4.0) * 2.0
    dmp.set_constants(alpha_y, beta_y)
    dmp.imitate(T, Y)
    dmp.configure(start_y=Y[0], goal_y=Y[-1])
    # fdmp = DMP(n_dims=6, execution_time=execution_time, dt=dt, n_weights_per_dim=args.force_bfs)
    # fdmp.imitate(T, Y_f)
    # fdmp.configure(start_y=Y_f[0], goal_y=Y_f[-1])

    pkl.dump(dmp, open('hybrid_control/saved_dmps/' + args.save_name + '_pos.pkl', 'wb'))
    pkl.dump(fdmp, open('hybrid_control/saved_dmps/' + args.save_name + '_force.pkl', 'wb'))

    

