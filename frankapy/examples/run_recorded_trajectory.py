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
import matplotlib.pyplot as plt

if __name__ == "__main__":
    fa = FrankaArm()
    # fa.reset_joints()

    rospy.loginfo('Generating Trajectory')

    pose_traj = pkl.load(open('examples/franka_traj.pkl','rb'))
    path = [pose.position for pose in pose_traj]
    path = np.array(path)
    forces = []

    T = 10
    dt = 0.01
    ts = np.arange(0, T, dt)

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    # ax.plot(path[:,0],path[:,1],path[:,2],color='b', label="commanded pos")

    rospy.loginfo('Initializing Sensor Publisher')
    pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=10)
    rate = rospy.Rate(1 / dt)

    rospy.loginfo('Publishing pose trajectory...')
    fa.goto_pose(pose_traj[0], duration=3, dynamic=False, buffer_time=3,
        cartesian_impedances=[600.0, 600.0, 600.0, 50.0, 50.0, 50.0]
    )
    # To ensure skill doesn't end before completing trajectory, make the buffer time much longer than needed
    fa.goto_pose(pose_traj[1], duration=T, dynamic=True, buffer_time=10,
        cartesian_impedances=[600.0, 600.0, 600.0, 50.0, 50.0, 50.0]
    )
    init_time = rospy.Time.now().to_time()
    
    curr_pose = fa.get_pose()
    actual_pos = [curr_pose.position]
    forces.append(fa.get_ee_force_torque())

    for i in range(2, len(ts)):
        curr_pose = fa.get_pose()
        actual_pos.append(curr_pose.position)
        forces.append(fa.get_ee_force_torque())
        timestamp = rospy.Time.now().to_time() - init_time
        traj_gen_proto_msg = PosePositionSensorMessage(
            id=i, timestamp=timestamp,
            position=pose_traj[i].translation, quaternion=pose_traj[i].quaternion
		)
        ros_msg = make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                traj_gen_proto_msg, SensorDataMessageType.POSE_POSITION),
            )

        rospy.loginfo('Publishing: ID {}'.format(traj_gen_proto_msg.id))
        pub.publish(ros_msg)
        rate.sleep()

    actual_pos = np.array(actual_pos)
    forces = np.array(forces)
    forces = forces * -0.002
    ax.plot(actual_pos[:,0],actual_pos[:,1],actual_pos[:,2],color='r', label='actual_pos')
    ax.quiver(actual_pos[:,0],actual_pos[:,1],actual_pos[:,2],forces[:,0],forces[:,1],forces[:,2])
    ax.set_ylim(-0.1,0.1)
    
    
    # Stop the skill
    # Alternatively can call fa.stop_skill()
    term_proto_msg = ShouldTerminateSensorMessage(timestamp=rospy.Time.now().to_time() - init_time, should_terminate=True)
    ros_msg = make_sensor_group_msg(
        termination_handler_sensor_msg=sensor_proto2ros_msg(
            term_proto_msg, SensorDataMessageType.SHOULD_TERMINATE)
        )
    pub.publish(ros_msg)

    rospy.loginfo('Done')

    plt.legend()
    plt.show()
