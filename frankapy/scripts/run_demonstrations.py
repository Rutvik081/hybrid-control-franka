import pickle as pkl
import numpy as np

from autolab_core import RigidTransform
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage, CartesianImpedanceSensorMessage
from franka_interface_msgs.msg import SensorDataGroup

from frankapy.utils import min_jerk, min_jerk_weight

import rospy
import random

def compute_aug_traj(pose_traj):
    positions = [pose.position for pose in pose_traj]
    positions = np.array(positions)
    dx = random.uniform(0.45 - positions[-1][0], 0.77 - positions[-1][0])
    dy = random.uniform(-0.05 - positions[-1][1], 0.13 - positions[-1][1])
    d = np.array([dx,dy,0])

    w = (positions - positions[0]) / (positions[-1] - positions[0])
    aug_positions = w*d + positions

    aug_traj = np.copy(pose_traj)
    for i in range(aug_traj.shape[0]):
        aug_traj[i].position = aug_positions[i]
    
    return aug_traj

if __name__ == '__main__':

    fa = FrankaArm()
    fa.reset_joints()

    rospy.loginfo('Generating Trajectory')
    
    pose_traj = pkl.load(open('scripts/franka_traj.pkl','rb'))
    pose_traj = np.array(pose_traj)

    aug_traj = compute_aug_traj(pose_traj[200:300,0])

    T = 10
    dt = 0.1
    ts = np.arange(0, T, dt)

    rospy.loginfo('Initializing Sensor Publisher')
    pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=10)
    rate = rospy.Rate(1 / dt)

    rospy.loginfo('Publishing pose trajectory...')
    # To ensure skill doesn't end before completing trajectory, make the buffer time much longer than needed
    fa.goto_pose(aug_traj[1], duration=T, dynamic=True, buffer_time=10,
        cartesian_impedances=[600.0, 600.0, 600.0, 50.0, 50.0, 50.0]
    )
    init_time = rospy.Time.now().to_time()

    for i in range(2, len(ts)):
        timestamp = rospy.Time.now().to_time() - init_time
        traj_gen_proto_msg = PosePositionSensorMessage(
            id=i, timestamp=timestamp,
            position=aug_traj[i].translation, quaternion=aug_traj[i].quaternion
		)
        ros_msg = make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                traj_gen_proto_msg, SensorDataMessageType.POSE_POSITION),
            )

        # rospy.loginfo('Publishing: ID {}'.format(traj_gen_proto_msg.id))
        print(aug_traj[i].position, fa.get_pose().position)
        pub.publish(ros_msg)
        rate.sleep()
    
    term_proto_msg = ShouldTerminateSensorMessage(timestamp=rospy.Time.now().to_time() - init_time, should_terminate=True)
    ros_msg = make_sensor_group_msg(
        termination_handler_sensor_msg=sensor_proto2ros_msg(
            term_proto_msg, SensorDataMessageType.SHOULD_TERMINATE)
        )
    pub.publish(ros_msg)

    rospy.loginfo('Done')

