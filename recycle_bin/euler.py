import time

import numpy as np
from teleop_controls.oculus_reader.oculus_reader.reader import OculusReader

from teleop_controls.misc.subprocess_utils import run_threaded_command
from teleop_controls.misc.transformations import add_angles, euler_to_quat, quat_diff, quat_to_euler, rmat_to_quat, axis_to_quat


def vec_to_reorder_mat(vec):
    X = np.zeros((len(vec), len(vec)))
    for i in range(X.shape[0]):
        ind = int(abs(vec[i])) - 1
        X[i, ind] = np.sign(vec[i])
    return X


class VRPolicy:
    def __init__(
        self,
        right_controller: bool = True,
        max_lin_vel: float = 1,
        max_rot_vel: float = 1,
        max_gripper_vel: float = 1,
        spatial_coeff: float = 1,
        pos_action_gain: float = 1,
        rot_action_gain: float = 0.1,
        gripper_action_gain: float = 255, #for RQGripper open and close reading covertsion from Boolean 1:0
        rmat_reorder: list = [1, -2, -3, 4], #have to do with conversion reording and daaformat from oculus 
    ):
        self.oculus_reader = OculusReader()
        self.vr_to_global_mat = np.eye(4)
        self.max_lin_vel = max_lin_vel
        self.max_rot_vel = max_rot_vel
        self.max_gripper_vel = max_gripper_vel
        self.spatial_coeff = spatial_coeff
        self.pos_action_gain = pos_action_gain
        self.rot_action_gain = rot_action_gain
        self.gripper_action_gain = gripper_action_gain
        self.global_to_env_mat = vec_to_reorder_mat(rmat_reorder)
        self.controller_id = "r" if right_controller else "l"
        self.reset_orientation = True
        self.reset_state()

        # Start State Listening Thread #
        run_threaded_command(self._update_internal_state)

    def reset_state(self):
        self._state = {
            "poses": {},
            "buttons": {"A": False, "B": False, "X": False, "Y": False},
            "movement_enabled": False,
            "controller_on": True,
        }
        self.update_sensor = True
        self.reset_origin = True
        self.robot_origin = None
        self.vr_origin = None
        self.vr_state = None

    def _update_internal_state(self, num_wait_sec=5, hz=50):
        last_read_time = time.time()
        while True:
            # Regulate Read Frequency #
            time.sleep(1 / hz)

            # Read Controller
            time_since_read = time.time() - last_read_time
            poses, buttons = self.oculus_reader.get_transformations_and_buttons()
            self._state["controller_on"] = time_since_read < num_wait_sec #turn off after 5 sec
            if poses == {}:
                continue

            # Determine Control Pipeline #
            toggled = self._state["movement_enabled"] != buttons[self.controller_id.upper() + "G"] #toggled is true if move_enabled does not match state of RG
            self.update_sensor = self.update_sensor or buttons[self.controller_id.upper() + "G"] #update sensor if update_senor flag or the button rg is true
            self.reset_orientation = self.reset_orientation or buttons[self.controller_id.upper() + "J"] #prssing joystick puts reset orientation to true
            self.reset_origin = self.reset_origin or toggled 

            # Save Info #
            self._state["poses"] = poses
            self._state["buttons"] = buttons
            self._state["movement_enabled"] = buttons[self.controller_id.upper() + "G"]
            self._state["controller_on"] = True
            last_read_time = time.time()

            # Update Definition Of "Forward" #
            stop_updating = self._state["buttons"][self.controller_id.upper() + "J"] or self._state["movement_enabled"] #rg or move enabled is true
            if self.reset_orientation:
                rot_mat = np.asarray(self._state["poses"][self.controller_id])
                if stop_updating:
                    self.reset_orientation = False
                # try to invert the rotation matrix, if not possible, then just use the identity matrix
                try:
                    rot_mat = np.linalg.inv(rot_mat)
                except:
                    print(f"exception for rot mat: {rot_mat}")
                    rot_mat = np.eye(4)
                    self.reset_orientation = True
                self.vr_to_global_mat = rot_mat

    def _process_reading(self): #turns 4x4 into vr_pos and vr_quat +trigger for gipper 
        rot_mat = np.asarray(self._state["poses"][self.controller_id])
        # test multiplying by another transformation matrix to rotate the frame
        rot_mat = self.global_to_env_mat @ self.vr_to_global_mat @ rot_mat
        vr_pos = self.spatial_coeff * rot_mat[:3, 3] #[[.][.][.]]
        vr_quat = rmat_to_quat(rot_mat[:3, :3])
        vr_gripper = self._state["buttons"]["RTr" if self.controller_id == "r" else "LTr"] #Boolean Gripper 

        self.vr_state = {"pos": vr_pos, "quat": vr_quat, "gripper": vr_gripper} 

    def _limit_velocity(self, lin_vel, rot_vel, gripper_vel): #no gripper limit needed keep it as is for boolean 
        """Scales down the linear and angular magnitudes of the action"""
        lin_vel_norm = np.linalg.norm(lin_vel)
        rot_vel_norm = np.linalg.norm(rot_vel)
        # gripper_vel_norm = np.linalg.norm(gripper_vel)
        if lin_vel_norm > self.max_lin_vel:
            lin_vel = lin_vel * self.max_lin_vel / lin_vel_norm
        if rot_vel_norm > self.max_rot_vel:
            rot_vel = rot_vel * self.max_rot_vel / rot_vel_norm
        # if gripper_vel_norm > self.max_gripper_vel:
        #     gripper_vel = gripper_vel * self.max_gripper_vel / gripper_vel_norm
        return lin_vel, rot_vel, gripper_vel

    def _calculate_action(self, state_dict, include_info=False):
        # Read Sensor #
        if self.update_sensor:
            self._process_reading()
            self.update_sensor = False

        # Read Observation
        robot_pos = np.array(state_dict["cartesian_position"][:3]) #read position
        robot_euler = state_dict["cartesian_position"][3:] 
        robot_quat = euler_to_quat(robot_euler) #convert reading into quat
        # robot_axis = state_dict["cartesian_position"][3:] 
        # robot_quat = axis_to_quat(robot_axis)
        #robot_gripper = state_dict["gripper_position"] ##perhaps this need to be adjusted for boolean processing but that is handled on the robotenv side

        # Reset Origin On Release #
        if self.reset_origin:
            self.robot_origin = {"pos": robot_pos, "quat": robot_quat}
            self.vr_origin = {"pos": self.vr_state["pos"], "quat": self.vr_state["quat"]}
            self.reset_origin = False

        # Calculate Positional Action #
        robot_pos_offset = robot_pos - self.robot_origin["pos"]
        target_pos_offset = self.vr_state["pos"] - self.vr_origin["pos"]
        pos_action = target_pos_offset - robot_pos_offset #delta pos from robot to desired position

        # Calculate Euler Action #
        robot_quat_offset = quat_diff(robot_quat, self.robot_origin["quat"])
        target_quat_offset = quat_diff(self.vr_state["quat"], self.vr_origin["quat"])
        quat_action = quat_diff(target_quat_offset, robot_quat_offset)
        euler_action = quat_to_euler(quat_action) 

        # Calculate Gripper Action #
        gripper_action = self.vr_state["gripper"] # boolean gripper now 

        # Calculate Desired Pose #
        target_pos = pos_action + robot_pos
        target_euler = add_angles(euler_action, quat_to_euler(robot_quat))
        # target_axis = add_angles(euler_action, robot_axis)

        target_cartesian = np.concatenate([target_pos, target_euler])
        # target_cartesian = np.concatenate([target_pos, target_axis])
        
        target_gripper = self.vr_state["gripper"] #target_gripper is boolean here 

        # Scale Appropriately #
        pos_action *= self.pos_action_gain 
        euler_action *= self.rot_action_gain
        #gripper_action *= self.gripper_action_gain #optionally Gripper action can bbe times by 255 so that 0 and 255 corspond to open or close respectively.
        #lin_vel, rot_vel, gripper_vel = self._limit_velocity(pos_action, euler_action, gripper_action)

        # Prepare Return Values #

        info_dict = {"target_cartesian_position": target_cartesian, "target_gripper_position": target_gripper}
        action = np.concatenate([pos_action, euler_action, [gripper_action]])  # position control

        #action = np.concatenate([lin_vel, rot_vel, [gripper_vel]])  velocity control 
        # action = action.clip(-1, 1)

        # Return #
        if include_info:
            return action, info_dict
        else:
            return action

    def get_info(self):
        return {
            "success": self._state["buttons"]["A"] if self.controller_id == 'r' else self._state["buttons"]["X"],
            "failure": self._state["buttons"]["B"] if self.controller_id == 'r' else self._state["buttons"]["Y"],
            "movement_enabled": self._state["movement_enabled"],
            "controller_on": self._state["controller_on"],
        }

    def forward(self, obs_dict, include_info=False):
        if self._state["poses"] == {}:
            action = np.zeros(7)
            if include_info:
                return action, {}
            else:
                return action
        return self._calculate_action(obs_dict, include_info=include_info)
