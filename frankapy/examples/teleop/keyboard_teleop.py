from pynput import keyboard

from scipy.spatial.transform import Rotation
import numpy as np

from multiprocessing.connection import Client, Listener
import pickle as pkl

import subprocess
import sys
import time
import traceback

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
                's': 'x+',
                'w': 'x-',
                'd': 'y+',
                'a': 'y-',
                'e': 'z+',
                'q': 'z-',
                'k': 'r+',
                'i': 'r-',
                'j': 'p+',
                'l': 'p-',
                'o': 'w+',
                'u': 'w-',
                }
        self.debug = False
        self.traj_port = 6000
        self.traj_authkey = b"trajectory"
        self.state_port = 6001
        self.state_authkey = b"robotstate"
        self.polling_rate_ms = 1.
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
            address = ("localhost", self.state_port)
            listener = Listener(address=address, authkey=self.state_authkey)
            conn = listener.accept()

            time.sleep(1.)
            address = ("localhost", self.traj_port)
            client = Client(address, authkey=self.traj_authkey)

            while not conn.poll():
                pass
            data = None
            while conn.poll():
                data = conn.recv()
            if not data:
                print("Something went wrong")
                sys.exit(1)
            msg = pkl.loads(data)
            current_pose, _ = msg


            while True:
                #data = None
                #while conn.poll():
                #    data = conn.recv()
                #if not data:
                #    time.sleep(self.polling_rate_ms * 1e-3) # roughly 1ms polling, with drift
                #    continue
                #msg = pkl.loads(data)
                #current_pose, _ = msg

                final_trans = current_pose[:3]
                t_scaling = 0.0001
                final_trans[0] += int(self.input_state['x+']) * t_scaling
                final_trans[0] -= int(self.input_state['x-']) * t_scaling
                final_trans[1] += int(self.input_state['y+']) * t_scaling
                final_trans[1] -= int(self.input_state['y-']) * t_scaling
                final_trans[2] += int(self.input_state['z+']) * t_scaling
                final_trans[2] -= int(self.input_state['z-']) * t_scaling

                r_scaling = 0.001
                new_rot = [0,0,0]
                new_rot[0] += int(self.input_state['r+']) * r_scaling
                new_rot[0] -= int(self.input_state['r-']) * r_scaling
                new_rot[1] += int(self.input_state['p+']) * r_scaling
                new_rot[1] -= int(self.input_state['p-']) * r_scaling
                new_rot[2] += int(self.input_state['w+']) * r_scaling
                new_rot[2] -= int(self.input_state['w-']) * r_scaling
                cur_R = Rotation.from_rotvec(current_pose[3:], degrees=False)
                new_R = Rotation.from_rotvec(new_rot, degrees=False)
                final_rot  = (cur_R * new_R).as_rotvec()

                target_pose = final_trans + final_rot.tolist()
                msg = (target_pose, _)
                data = pkl.dumps(msg)
                client.send(data)
                current_pose = target_pose
                time.sleep(self.polling_rate_ms * 1e-3) # roughly 1ms polling, with drift
        except:
            traceback.print_exc()
        finally:
            show_keystrokes()

if __name__ == '__main__':
    keyboard_teleop = KeyboardTeleop()
    keyboard_teleop.start_teleop()
