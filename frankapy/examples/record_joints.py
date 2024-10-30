from frankapy import FrankaArm
import pickle as pkl

fa = FrankaArm()

pkl.dump(fa.get_joints(),open('examples/marker_joints.pkl', 'wb'))