import pickle as pkl
import argparse
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.signal import savgol_filter
import sys
from cs import CanonicalSystem

from force_dmp import ForceDMP

import matplotlib.pyplot as plt

from movement_primitives.dmp import CartesianDMP

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--dmp_name', '-dn')
    args = parser.parse_args()

    dmp = pkl.load(open('hybrid_control/saved_dmps/' + args.dmp_name + '_pos.pkl', 'rb'))
    fdmp = pkl.load(open('hybrid_control/saved_dmps/' + args.dmp_name + '_force.pkl', 'rb'))

    pos_traj = pkl.load(open('hybrid_control/saved_demos/' + args.dmp_name + '_pose_traj.pkl', 'rb'))
    pose_demo = [pose.position for pose in pos_traj]
    pose_demo = np.array(pose_demo)

    force_demo = pkl.load(open('hybrid_control/saved_demos/' + args.dmp_name + '_target_forces.pkl', 'rb'))
    force_demo = np.array(force_demo)

    tau = 0.5
    T = fdmp['x'].T * tau
    dmp.set_execution_time_(T)
    start_y = dmp.start_y[:3]
    goal_y = dmp.goal_y[:3]
    # goal_y[1] += 0.04
    _, Y = dmp.open_loop(start_y=start_y, goal_y=goal_y)
    # T_f, Y_f = fdmp.open_loop()

    x_dmp = Y[:, 0]
    y_dmp = Y[:, 1]
    z_dmp = Y[:, 2]
    
    x = pose_demo[:,0]
    y = pose_demo[:,1]
    z = pose_demo[:,2]

    rot_matrices = [pose.rotation for pose in pos_traj]
    rot_matrices = np.array(rot_matrices)
    u = rot_matrices[:,0][:,0]
    v = rot_matrices[:,0][:,1]
    w = rot_matrices[:,0][:,2]

    rot = Rotation.from_quat(np.roll(Y[:, 3:], -1, axis=1))
    rot_matrices_dmp = rot.as_matrix()
    u_dmp = rot_matrices_dmp[:,0][:,0]
    v_dmp = rot_matrices_dmp[:,0][:,1]
    w_dmp = rot_matrices_dmp[:,0][:,2]

    tau = 2.0
    T = fdmp['x'].T * tau
    dt = fdmp['x'].dt
    a = 1.0
    fdmp['x'].cs = CanonicalSystem(a, T, dt)
    fdmp['y'].cs = CanonicalSystem(a, T, dt)
    fdmp['z'].cs = CanonicalSystem(a, T, dt)
    fdmp['ro'].cs = CanonicalSystem(a, T, dt)
    fdmp['pi'].cs = CanonicalSystem(a, T, dt)
    fdmp['ya'].cs = CanonicalSystem(a, T, dt)
    fx_dmp = fdmp['x'].run_sequence(tau)
    fy_dmp = fdmp['y'].run_sequence(tau)
    fz_dmp = fdmp['z'].run_sequence(tau)
    fro_dmp = fdmp['ro'].run_sequence(tau)
    fpi_dmp = fdmp['pi'].run_sequence(tau)
    fya_dmp = fdmp['ya'].run_sequence(tau)

    # fx_dmp = Y_f[:,0]
    # fy_dmp = Y_f[:,1]
    # fz_dmp = Y_f[:,2]

    fx = force_demo[:,0]
    fy = force_demo[:,1]
    fz = force_demo[:,2]
    fro = force_demo[:,3]
    fpi = force_demo[:,4]
    fya = force_demo[:,5]

    fx_filter = savgol_filter(fx, 51, 3)
    fy_filter = savgol_filter(fy, 51, 3)
    fz_filter = savgol_filter(fz, 51, 3)
    fro_filter = savgol_filter(fro, 51, 3)
    fpi_filter = savgol_filter(fpi, 51, 3)
    fya_filter = savgol_filter(fya, 51, 3)

    # fig1, ((ax1), (ax2), (ax3)) = plt.subplots(3, 1, figsize=(10,8))
    # ax1.plot(fx_dmp)
    # ax1.set_ylabel('fx_dmp')
    # ax2.plot(fy_dmp)
    # ax2.set_ylabel('fy_dmp')
    # ax3.plot(fz_dmp)
    # ax3.set_ylabel('fz_dmp')
    
    # fig1, ((ax1), (ax2), (ax3)) = plt.subplots(3, 1, figsize=(10,8))
    # ax1.plot(fro_dmp)
    # ax1.set_ylabel('fro_dmp')
    # ax2.plot(fpi_dmp)
    # ax2.set_ylabel('fpi_dmp')
    # ax3.plot(fya_dmp)
    # ax3.set_ylabel('fya_dmp')

    # fig1, ((ax1), (ax2), (ax3)) = plt.subplots(3, 1, figsize=(10,8))
    # ax1.plot(fx_filter)
    # ax1.set_ylabel('fx_filter')
    # ax2.plot(fy_filter)
    # ax2.set_ylabel('fy_filter')
    # ax3.plot(fz_filter)
    # ax3.set_ylabel('fz_filter')

    # fig1, ((ax1), (ax2), (ax3)) = plt.subplots(3, 1, figsize=(10,8))
    # ax1.plot(fro_filter)
    # ax1.set_ylabel('fro_filter')
    # ax2.plot(fpi_filter)
    # ax2.set_ylabel('fpi_filter')
    # ax3.plot(fya_filter)
    # ax3.set_ylabel('fya_filter')

    # fig2, ((ax1), (ax2), (ax3)) = plt.subplots(3, 1, figsize=(10,8))
    # ax1.plot(fx)
    # ax1.set_ylabel('fx')
    # ax2.plot(fy)
    # ax2.set_ylabel('fy')
    # ax3.plot(fz)
    # ax3.set_ylabel('fz')

    # fig2, ((ax1), (ax2), (ax3)) = plt.subplots(3, 1, figsize=(10,8))
    # ax1.plot(fro)
    # ax1.set_ylabel('fro')
    # ax2.plot(fpi)
    # ax2.set_ylabel('fpi')
    # ax3.plot(fya)
    # ax3.set_ylabel('fya')

    # fig3, ((ax1), (ax2), (ax3)) = plt.subplots(3, 1, figsize=(10,8))
    # ax1.plot(x_dmp)
    # ax1.set_ylabel('x_dmp')
    # ax2.plot(y_dmp)
    # ax2.set_ylabel('y_dmp')
    # ax3.plot(z_dmp)
    # ax3.set_ylabel('z_dmp')

    # fig4, ((ax1), (ax2), (ax3)) = plt.subplots(3, 1, figsize=(10,8))
    # ax1.plot(x)
    # ax1.set_ylabel('x')
    # ax2.plot(y)
    # ax2.set_ylabel('y')
    # ax3.plot(z)
    # ax3.set_ylabel('z')

    ax = plt.figure().add_subplot(projection='3d')
    ax.quiver(x[::5], y[::5], z[::5], u[::5], v[::5], w[::5], length=0.02, normalize=True)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_zlim(-0.5,0.5)

    ax = plt.figure().add_subplot(projection='3d')
    ax.quiver(x_dmp[::5], y_dmp[::5], z_dmp[::5], u_dmp[::5], v_dmp[::5], w_dmp[::5], length=0.02, normalize=True)
    ax.set_xlabel('x_dmp')
    ax.set_ylabel('y_dmp')
    ax.set_zlabel('z_dmp')
    ax.set_zlim(-0.5,0.5)

    # ax = plt.figure().add_subplot(projection='3d')
    # ax.quiver(x_dmp[::10], y_dmp[::10], z_dmp[::10], fx_dmp[::10], fy_dmp[::10], fz_dmp[::10], length=0.02, normalize=True)
    # ax.set_xlabel('x_dmp')
    # ax.set_ylabel('y_dmp')
    # ax.set_zlabel('z_dmp')
    # ax.set_zlim(-0.5,0.5)

    
    # ax = plt.figure().add_subplot(projection='3d')
    # ax.quiver(x_dmp[::10][1:], y_dmp[::10][1:], z_dmp[::10][1:], fx[::10], fy[::10], fz[::10], length=0.02, normalize=True)
    # ax.set_xlabel('x')
    # ax.set_ylabel('y')
    # ax.set_zlabel('z')
    # ax.set_zlim(-0.5,0.5)

    plt.show()
    