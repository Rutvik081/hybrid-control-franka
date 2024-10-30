from frankapy import FrankaArm
import pickle as pkl
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--joints_name', '-jn')
    args = parser.parse_args()

    fa = FrankaArm()
    joints = fa.get_joints()
    pkl.dump(joints, open('hybrid_control/saved_joints/' + args.joints_name + '.pkl', 'wb'))
