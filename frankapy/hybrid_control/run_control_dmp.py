import argparse
import pickle as pkl
from force_dmp import ForceDMP
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import ForcePositionSensorMessage, ForcePositionControllerSensorMessage
from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage, CartesianImpedanceSensorMessage
from franka_interface_msgs.msg import SensorDataGroup
from frankapy.utils import transform_to_list
import rospy
from scipy.spatial.transform import Rotation
import time
import numpy as np
import sys
from scipy.signal import savgol_filter
from time import sleep
from cs import CanonicalSystem

sys.path.append('/opt/frankapy/examples')
from movement_primitives.dmp import CartesianDMP

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--dmp_name', '-dn')
    parser.add_argument('--start_joints', '-sj')
    parser.add_argument('--time_scaling', '-ts')
    parser.add_argument('--goal_offset_x', '-gox', default='0.0')
    parser.add_argument('--goal_offset_y', '-goy', default='0.0')
    args = parser.parse_args()

    fa = FrankaArm()
    rospy.loginfo('Generating Trajectory')
    dmp = pkl.load(open('examples/hybrid_control/saved_dmps/' + args.dmp_name + '_pos.pkl', 'rb'))
    fdmp = pkl.load(open('examples/hybrid_control/saved_dmps/' + args.dmp_name + '_force.pkl', 'rb'))
    force_demo = pkl.load(open('examples/hybrid_control/saved_demos/' + args.dmp_name + '_target_forces.pkl', 'rb'))
    force_demo = np.array(force_demo)

    force_demo[:,0] = savgol_filter(force_demo[:,0], 51, 3)
    force_demo[:,1] = savgol_filter(force_demo[:,1], 51, 3)
    force_demo[:,2] = savgol_filter(force_demo[:,2], 51, 3)
    force_demo[:,3] = savgol_filter(force_demo[:,3], 51, 3)
    force_demo[:,4] = savgol_filter(force_demo[:,4], 51, 3)
    force_demo[:,5] = savgol_filter(force_demo[:,5], 51, 3)

    T = dmp.get_execution_time_()
    T = T * float(args.time_scaling)
    dt = dmp.dt_
    ts = np.arange(0, T, dt)
    N = len(ts)
    position_kps_cart = [600.0, 600.0, 600.0, 25.0, 25.0, 25.0]
    force_kps_cart = [0.1] * 6
    S = [0,0,0,0,0,0]
    f_th = 3.0
    a = 1.0

    dmp.set_execution_time_(T)
    start_y = dmp.start_y[:3]
    goal_y = dmp.goal_y[:3]
    goal_y[0] += float(args.goal_offset_x)    # -0.059              [0.438, -0.096, 0.058]  [0.478, -0.108, 0.058]
    goal_y[1] += float(args.goal_offset_y)    # 0.018               [-0.04, 0.012, 0]
    fdmp['x'].cs = CanonicalSystem(a, T, dt)
    fdmp['y'].cs = CanonicalSystem(a, T, dt)
    fdmp['z'].cs = CanonicalSystem(a, T, dt)
    fdmp['ro'].cs = CanonicalSystem(a, T, dt)
    fdmp['pi'].cs = CanonicalSystem(a, T, dt)
    fdmp['ya'].cs = CanonicalSystem(a, T, dt)

    rospy.loginfo('Initializing Sensor Publisher')
    pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1)
    hz = 1./dt
    rate = rospy.Rate(hz)

    start_joints = pkl.load(open('examples/hybrid_control/saved_joints/' + args.start_joints + '.pkl', 'rb'))
    fa.goto_joints(start_joints)

    pos = fa.get_pose()

    pos.translation = dmp.start_y[:3]
    rot = Rotation.from_quat(np.roll(dmp.start_y[3:], -1))
    pos.rotation = rot.as_matrix().reshape(3,3)

    fa.run_modified_force_position(duration=T, buffer_time=3, S=S,
                                use_cartesian_gains=True,
                                position_kps_cart=position_kps_cart,
                                force_kps_cart=force_kps_cart)
    
    init_time = rospy.Time.now().to_time()

    last_y = dmp.start_y
    last_yd = np.zeros(6)

    fx = fdmp['x'].step(tau=float(args.time_scaling))
    fy = fdmp['y'].step(tau=float(args.time_scaling))
    fz = fdmp['z'].step(tau=float(args.time_scaling))
    fro = fdmp['ro'].step(tau=float(args.time_scaling))
    fpi = fdmp['pi'].step(tau=float(args.time_scaling))
    fya = fdmp['ya'].step(tau=float(args.time_scaling))

    for i in range(N):

        timestamp = rospy.Time.now().to_time() - init_time
        
        target_force = np.array([fx, fy, fz, fro, fpi, fya])
        curr_force = fa.get_ee_force_torque()
        f_error = curr_force - target_force
        f_error = np.linalg.norm(f_error[:3])
        f_des_mag = np.linalg.norm(target_force)
        print('desired_force_mag:', f_des_mag, 'error_mag:', f_error)

        if S == [0,0,0,0,0,0] and f_des_mag > 5.0:
            S = [1,1,1,1,1,1]
            print('switched to hybrid control')
        
        current_y, current_yd = dmp.step(last_y, last_yd, goal_y=goal_y)
        pos.translation = current_y[:3]

        rot = Rotation.from_quat(np.roll(current_y[3:], -1))
        pos.rotation = np.array(rot.as_matrix().reshape(3,3))

        traj_gen_proto_msg = ForcePositionSensorMessage(
            id=i, timestamp=timestamp, seg_run_time=dt,
            pose=transform_to_list(pos),
            force = target_force
        )
        fb_ctrlr_proto = ForcePositionControllerSensorMessage(
            id=i, timestamp=timestamp,
            position_kps_cart=position_kps_cart,
            force_kps_cart=force_kps_cart,
            selection=S
        )
        ros_msg = make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                traj_gen_proto_msg, SensorDataMessageType.FORCE_POSITION),
            feedback_controller_sensor_msg=sensor_proto2ros_msg(
                fb_ctrlr_proto, SensorDataMessageType.FORCE_POSITION_GAINS)
        )
        fx = fdmp['x'].step(tau=float(args.time_scaling))
        fy = fdmp['y'].step(tau=float(args.time_scaling))
        fz = fdmp['z'].step(tau=float(args.time_scaling))
        fro = fdmp['ro'].step(tau=float(args.time_scaling))
        fpi = fdmp['pi'].step(tau=float(args.time_scaling))
        fya = fdmp['ya'].step(tau=float(args.time_scaling))
        last_y = current_y
        last_yd = current_yd
        
        pub.publish(ros_msg)
        time.sleep(dt)
        rate.sleep()
    
    fa.stop_skill()
    rospy.loginfo('Done')
