import time
from frankapy import FrankaArm
import argparse
import threading
import pickle as pkl

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--num_demonstrations', '-n', type=int, default=1)
    parser.add_argument('--file', '-f', default='scripts/franka_traj.pkl')
    parser.add_argument('--time', '-t', type=float, default=10)
    args = parser.parse_args()

    print("Starting robot")
    fa = FrankaArm()
    fa.reset_joints()

    end_effector_position = []
    for i in range(args.num_demonstrations):
        print('Recording demonstrations ({}/{})'.format(i+1,args.num_demonstrations))
        terminated = False
        fa.reset_joints()
        fa.run_guide_mode(args.time, block=False)
        for i in range(100):
            if i == 99:
                terminated = True
            end_effector_position.append([fa.get_pose(),terminated])
            time.sleep(0.1)
        input("Press Enter to record next demonstration...")
    
    pkl.dump(end_effector_position, open(args.file, 'wb'))


