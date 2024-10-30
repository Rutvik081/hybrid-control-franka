from frankapy import FrankaArm
import pickle as pkl
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--joints_name', '-sn')
    args = parser.parse_args()

    fa = FrankaArm()
    joints = pkl.load(open('hybrid_control/saved_joints/' + args.joints_name + '.pkl', 'rb'))
    fa.goto_joints(joints)