from pynput import keyboard

from scipy.spatial.transform import Rotation
import numpy as np
import pickle as pkl
import subprocess
import traceback

from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto import PosePositionSensorMessage
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from franka_interface_msgs.msg import SensorDataGroup
import rospy
import matplotlib.pyplot as plt
import struct
import time

def calculate_forces(curr_pose, des_trans, des_quat, position_kps_cart):
    pos_err = curr_pose.translation - des_trans
    R_c = Rotation.from_quat(np.roll(curr_pose.quaternion,-1))
    R_d = Rotation.from_quat(np.roll(des_quat,-1))
    rot_err = (R_c * R_d.inv()).as_rotvec()
    total_error = np.concatenate((pos_err,rot_err),axis=0)
    position_kps_cart = np.array(position_kps_cart)
    return (- np.multiply(position_kps_cart,total_error)).tolist()

def hide_keystrokes():
    subprocess.run(['stty', '-echo'], check=True)
def show_keystrokes():
    subprocess.run(['stty', 'echo'], check=True)

class KeyboardTeleop():
    def __init__(self):
        self.input_state = {
                'x+': False,
                'x-': False,
                'y+': False,
                'y-': False,
                'z+': False,
                'z-': False,
                'r+': False,
                'r-': False,
                'p+': False,
                'p-': False,
                'w+': False,
                'w-': False,
                }
        self.key_bindings = {
                'w': 'x+',
                's': 'x-',
                'a': 'y+',
                'd': 'y-',
                'q': 'z+',
                'e': 'z-',
                'l': 'r+',
                'j': 'r-',
                'i': 'p+',
                'k': 'p-',
                'o': 'w+',
                'u': 'w-',
                }
        self.debug = False
    
    def on_press(self, key):
        try:
            input_type = self.key_bindings[key.char]
            self.input_state[input_type] = True
        except AttributeError:
            if self.debug:
                print(f"special key {key} pressed.")
        except KeyError:
            if self.debug:
                print(f"Pressed key {key} is not bound.")
    
    def on_release(self, key):
        try:
            input_type = self.key_bindings[key.char]
            self.input_state[input_type] = False
        except AttributeError:
            if self.debug:
                print(f"special key {key} pressed.")
        except KeyError:
            if self.debug:
                print(f"Pressed key {key} is not bound.")
        finally:
            if key == keyboard.Key.esc:
                # Stop listener
                return False
    
    def start_teleop(self):
        listener = keyboard.Listener(
                on_press=self.on_press,
                on_release=self.on_release)
        listener.start()
        try:
            hide_keystrokes()
            
            fa = FrankaArm()
            T = 1000     # 20 40 95
            dt = 0.01
            ts = np.arange(0, T, dt)
            position_kps_cart = [600.0, 600.0, 600.0, 25.0, 25.0, 25.0]
            
            rospy.loginfo('Initializing Sensor Publisher')
            pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=10)
            rate = rospy.Rate(1 / dt)

            current_pose = fa.get_pose()
            
            fa.goto_pose(current_pose, duration=T, dynamic=True, buffer_time=10,
                cartesian_impedances=position_kps_cart
            )

            init_time = rospy.Time.now().to_time()

            current_trans = current_pose.translation
            current_rot = current_pose.rotation
            target_forces = []
            target_impedances = []
            pose_traj = []

            for i in range(len(ts)):

                final_trans = current_trans
                t_scaling = 0.0005
                final_trans[0] += int(self.input_state['x+']) * t_scaling
                final_trans[0] -= int(self.input_state['x-']) * t_scaling
                final_trans[1] += int(self.input_state['y+']) * t_scaling
                final_trans[1] -= int(self.input_state['y-']) * t_scaling
                final_trans[2] += int(self.input_state['z+']) * t_scaling
                final_trans[2] -= int(self.input_state['z-']) * t_scaling
                current_trans = final_trans

                r_scaling = 0.005
                new_rot = [0,0,0]
                new_rot[0] += int(self.input_state['r+']) * r_scaling
                new_rot[0] -= int(self.input_state['r-']) * r_scaling
                new_rot[1] += int(self.input_state['p+']) * r_scaling
                new_rot[1] -= int(self.input_state['p-']) * r_scaling
                new_rot[2] += int(self.input_state['w+']) * r_scaling
                new_rot[2] -= int(self.input_state['w-']) * r_scaling
                cur_R = Rotation.from_matrix(current_rot)
                new_R = Rotation.from_rotvec(new_rot, degrees=False)
                final_rot  = cur_R * new_R
                current_rot = final_rot.as_matrix()
                final_rot = final_rot.as_quat()
                final_rot = np.roll(final_rot,1)

                timestamp = rospy.Time.now().to_time() - init_time
                target_impedance = calculate_forces(fa.get_pose(),final_trans, final_rot,position_kps_cart)
                target_force = fa.get_ee_force_torque()
                target_forces.append(target_force)
                target_impedances.append(target_impedance)
                curr_pose = fa.get_pose()
                pose_traj.append(curr_pose)
                traj_gen_proto_msg = PosePositionSensorMessage(
                    id=i, timestamp=timestamp,
                    position=final_trans, quaternion=final_rot
                )
                ros_msg = make_sensor_group_msg(
                    trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                        traj_gen_proto_msg, SensorDataMessageType.POSE_POSITION),
                )

                # rospy.loginfo('Publishing: ID {}'.format(traj_gen_proto_msg.id))
                pub.publish(ros_msg)
                time.sleep(dt)
                rate.sleep()

            pkl.dump(target_forces, open('examples/demos/saved_demos/wrapping_old_target_forces.pkl', 'wb'))
            pkl.dump(target_impedances, open('examples/demos/saved_demos/wrapping_old_target_impedances.pkl', 'wb'))
            pkl.dump(pose_traj, open('examples/demos/saved_demos/wrapping_old_pose_traj.pkl', 'wb'))
            
            fa.stop_skill()
            rospy.loginfo('Done')
        except:
            traceback.print_exc()
        finally:
            show_keystrokes()

if __name__ == '__main__':
    keyboard_teleop = KeyboardTeleop()
    keyboard_teleop.start_teleop()