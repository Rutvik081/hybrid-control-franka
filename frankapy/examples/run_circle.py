import pickle as pkl
import numpy as np
import math
import matplotlib.pyplot as plt

from autolab_core import RigidTransform
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage, CartesianImpedanceSensorMessage
from franka_interface_msgs.msg import SensorDataGroup

from frankapy.utils import min_jerk, min_jerk_weight

import rospy

if __name__ == "__main__":
    fa = FrankaArm()
    fa.reset_joints()
    franka_pose = fa.get_pose()
    rospy.loginfo('Generating Trajectory')

    T = 5
    dt = 0.1
    ts = np.arange(0, T, dt)
    N = len(ts)
    r = 0.05
    xcs = np.array([0.5])  #np.array([0.3,0.5,0.7])
    ycs = np.array([0.0]) #np.array([-0.2,0.0,0.2])
    zcs = np.array([0.2]) #np.array([0.1,0.2,0.3])
    quat = np.array([0,1,0,0])

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    rospy.loginfo('Initializing Sensor Publisher')
    pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=10)
    rate = rospy.Rate(1 / dt)

    for zc in zcs:
        for xc in xcs:
            for yc in ycs:
                path = []
                for i in range(N):
                    x = xc + r * math.cos(((i+1)/N) * 2 * math.pi)  #+ (i+1)*0.4/N #
                    y = yc - r * math.sin(((i+1)/N) * 2 * math.pi)  #+ (i+1)*0.4/N #
                    path.append([x,y,zc])
                path = np.array(path)
                ax.plot(path[:,0],path[:,1],path[:,2],color='b', label="commanded pos")

                rospy.loginfo('Publishing pose trajectory...')
                # To ensure skill doesn't end before completing trajectory, make the buffer time much longer than needed
                franka_pose.translation = path[0]
                fa.goto_pose(franka_pose, duration=3, dynamic=False, buffer_time=3,
                    cartesian_impedances=[600.0, 600.0, 600.0, 50.0, 50.0, 50.0]
                )
                curr_pose = fa.get_pose()
                actual_pos = [curr_pose.position]

                franka_pose.translation = path[1]
                fa.goto_pose(franka_pose, duration=T, dynamic=True, buffer_time=T,
                    cartesian_impedances=[600.0, 600.0, 600.0, 50.0, 50.0, 50.0]
                )
                init_time = rospy.Time.now().to_time()
                curr_pose = fa.get_pose()
                actual_pos = [curr_pose.position]
                # actual_pos.append(curr_pose.position)

                for i in range(2, len(ts)):
                    curr_pose = fa.get_pose()
                    actual_pos.append(curr_pose.position)
                    timestamp = rospy.Time.now().to_time() - init_time
                    traj_gen_proto_msg = PosePositionSensorMessage(
                        id=i, timestamp=timestamp,
                        position=path[i], quaternion=np.array([0,1,0,0])
                    )
                    ros_msg = make_sensor_group_msg(
                        trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                            traj_gen_proto_msg, SensorDataMessageType.POSE_POSITION),
                        )

                    # rospy.loginfo('Publishing: ID {}'.format(traj_gen_proto_msg.id))
                    pub.publish(ros_msg)
                    rate.sleep()
                
                actual_pos = np.array(actual_pos)
                ax.plot(actual_pos[:,0],actual_pos[:,1],actual_pos[:,2],color='r', label='actual_pos')

                # Stop the skill
                # Alternatively can call fa.stop_skill()
                # term_proto_msg = ShouldTerminateSensorMessage(timestamp=rospy.Time.now().to_time() - init_time, should_terminate=True)
                # ros_msg = make_sensor_group_msg(
                #     termination_handler_sensor_msg=sensor_proto2ros_msg(
                #         term_proto_msg, SensorDataMessageType.SHOULD_TERMINATE)
                #     )
                # pub.publish(ros_msg)
                fa.stop_skill()
                rospy.loginfo('Done')


    # # fig = plt.figure()
    # # ax = fig.add_subplot(projection='3d')
    # # ax.plot(path[:,0],path[:,1],path[:,2],color='b')
    # # plt.show()


    

    
    # plt.legend()
    # plt.show()

    # # fig = plt.figure()
    # # ax = fig.add_subplot(projection='3d')
    # # ax.plot(path[:,0],path[:,1],path[:,2],color='b',label='circle')
    # # ax.plot(actual_pos[:,0],actual_pos[:,1],actual_pos[:,2],label='actual_pos')
    # ax.set_xlim(0,1)
    # ax.set_ylim(-0.5,0.5)
    # ax.set_zlim(0,0.5)
    # plt.show()