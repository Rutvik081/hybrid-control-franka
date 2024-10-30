import argparse
import time
from frankapy import FrankaArm
import pickle as pkl

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--time', '-t', type=float, default=1000)
    parser.add_argument('--open_gripper', '-o', action='store_true')
    parser.add_argument('--file', '-f', default='examples/franka_traj.pkl')
    args = parser.parse_args()

    print('Starting robot 123')
    fa = FrankaArm()
    # fa.open_gripper()
    # if args.open_gripper:
    #     fa.open_gripper()
    # fa.reset_joints()
    # start_joints = pkl.load(open('examples/start_joints.pkl','rb'))
    # fa.goto_joints(start_joints)
    print('Applying 0 force torque control for {}s'.format(args.time))
    end_effector_position = []
    eef_forces = []
    fa.run_guide_mode(args.time, block=False)

    for i in range(1000):
        # end_effector_position.append(fa.get_pose())
        # eef_forces.append(fa.get_ee_force_torque())
        time.sleep(0.01)

    # pkl.dump(end_effector_position, open(args.file, 'wb'))
    # pkl.dump(eef_forces, open('examples/franka_forces.pkl', 'wb'))