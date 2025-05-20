#!/usr/bin/env python
# coding: utf-8

# In[ ]:


# Making a very simple code to test oculus-to-robot deployment on UR10e

# initial imports
from teleop_controls.oculus_reader.oculus_reader.reader import OculusReader
from teleop_controls.misc.subprocess_utils import run_multiprocessed_command
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import time
import math
import pyrealsense2 as rs
import cv2
import numpy as np
# import jax
import robotiq_gripper
import PySpin
import matplotlib.pyplot as plt
# from PIL import Image
import datetime
from scipy.spatial.transform import Rotation as R
import os
import csv
# import math3d as m3d

# constants
ROBOT_IP = '192.168.0.110'
GRIPPER_PORT = 63352

# Parameters
rtde_frequency = 500.0
flags = RTDEControl.FLAG_VERBOSE | RTDEControl.FLAG_UPLOAD_SCRIPT
ur_cap_port = 50002

# ur_rtde realtime priorities
rt_receive_priority = 90
rt_control_priority = 85

# Initialize ur-rtde recieve and control commands
rtde_r = RTDEReceive(ROBOT_IP, rtde_frequency, [], True, False, rt_receive_priority)
rtde_c = RTDEControl(ROBOT_IP, rtde_frequency, flags, ur_cap_port, rt_control_priority)

# set robot joints start position
robot_startposition = (
    math.radians(252),
    math.radians(-77),
    math.radians(132),
    math.radians(161),
    math.radians(-67),
    math.radians(-22)
)

print("Start Position:", robot_startposition)

# Move robot to preset start position
rtde_c.moveJ(robot_startposition)

# initialize gripper
print("Creating gripper...")
gripper = robotiq_gripper.RobotiqGripper()
print("Connecting to gripper...")
gripper.connect(ROBOT_IP, GRIPPER_PORT)
print("Activating gripper...")
gripper.activate()


# In[ ]:


# initialize the realsense cameras
# first camera
# first camera is the one facing the user on the table
pipeline = rs.pipeline()
config = rs.config()
# specify camera serial number (this is the user camera)
config.enable_device('128422270567')
print(f"Device: Intel Realsense D405")
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
pipeline.start(config)
    
# second camera
# second camera displays the robot's wrist camera view for better depth perception
pipeline_robot = rs.pipeline()
config_robot = rs.config()
config_robot.enable_device('218622276856')
print(f"Device: Intel Realsense D405 - Robot Wrist Camera")
config_robot.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
config_robot.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
pipeline_robot.start(config_robot)

# initial frame fetch for display
# first camera:
firstpov_frames = pipeline.wait_for_frames()
color_frame = firstpov_frames.get_color_frame()
depth_frame = firstpov_frames.get_depth_frame()

# second camera:
secondpov_frames = pipeline_robot.wait_for_frames()
secondpov_color_frame = secondpov_frames.get_color_frame()
secondpov_depth_frame = secondpov_frames.get_depth_frame()

# if not depth_frame or not color_frame:
#     continue                        
firstpov_frame = np.asanyarray(color_frame.get_data())
depth_image = np.asanyarray(depth_frame.get_data())

# if not secondpov_color_frame or not secondpov_depth_frame:
#     continue
secondpov_frame = np.asanyarray(secondpov_color_frame.get_data())

# reshape image frames to be consistent with octo data
firstpov_frame = cv2.resize(np.array(firstpov_frame),(256,256))
secondpov_frame = cv2.resize(np.array(secondpov_frame),(256,256))


# In[ ]:


# primary view - pointgrey 3rd person camera

print("Getting primary camera set up...")

# retrieve camera
result = True
serial = '13440209'

system = PySpin.System.GetInstance()
camera_list = system.GetCameras()
num_cams = camera_list.GetSize()

if num_cams == 0:
    camera_list.Clear()
    print('No cameras found.')

# start camera stream
cam = camera_list.GetBySerial(serial)
cam.Init()

nodemap = cam.GetNodeMap()
nodemap_tldevice = cam.GetTLDeviceNodeMap()
sNodemap = cam.GetTLStreamNodeMap()
cam.PixelFormat.SetValue(PySpin.PixelFormat_BayerRG8)
print (f"Pixel format set to: {cam.PixelFormat.GetCurrentEntry().GetSymbolic()}")

# set acquisition mode to continuous
# first need to set value of enumeration node
node_acquisition_mode = PySpin.CEnumerationPtr(nodemap.GetNode("AcquisitionMode"))
if not PySpin.IsAvailable(node_acquisition_mode) or not PySpin.IsWritable(node_acquisition_mode):
    print ("Unable to set acquisition mode to continuous (enum retrieval). Aborting...")

# get entry node from enumeration node
node_acquisition_mode_continuous = node_acquisition_mode.GetEntryByName("Continuous")
if not PySpin.IsAvailable(node_acquisition_mode_continuous) or not PySpin.IsReadable(node_acquisition_mode_continuous):
    print ("Unable to set acquisition mode to continuous (entry retrieval). Aborting...")

# retrieve integer value from entry node
acquisition_mode_continuous = node_acquisition_mode_continuous.GetValue()

# set integer value from entry node as new value of enumeration node
node_acquisition_mode.SetIntValue(acquisition_mode_continuous)

cam.BeginAcquisition()

time.sleep(1) # allow some time for startup for camera

height = cam.Height()
width = cam.Width()

# create inital frames observation array
frames = cam.GetNextImage() # add a buffer to the frames

cv2_img = frames.GetData().reshape(height, width, 1)
cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_BAYER_RG2RGB)
primary_img = cv2.resize(np.array(cv2_img),(256,256))

# show all camera views stacked together horizontally
stacked_viewframes = np.hstack((cv2.flip(firstpov_frame, 1), secondpov_frame, primary_img))
cv2.imshow("Realsense", stacked_viewframes)


# In[ ]:


# function for applying transformation on the frame of reference later
# may need to edit this depending on our robot vs Franka
def vec_to_reorder_mat(vec):
    X = np.zeros((len(vec), len(vec)))
    for i in range(X.shape[0]):
        ind = int(abs(vec[i])) - 1
        X[i, ind] = np.sign(vec[i])
    return X


# In[ ]:


# class for vr policy; contains variables for calculating action
# borrowed some parts from DROID, with some modifications
class VRPolicy:
    def __init__(
        self,
        right_controller: bool = True,
        max_lin_vel: float = 1,
        max_rot_vel: float = 1,
        max_gripper_vel: float = 1,
        spatial_coeff: float = 1,
        pos_action_gain: float = 5,
        rot_action_gain: float = 2,
        gripper_action_gain: float = 3,
        rmat_reorder: list = [-2, -1, -3, 4],
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
        
        # for reading from the robot
        self.ur

        # Start State Listening Thread #
        run_threaded_command(self._update_internal_state)
    
    # method that resets the state of the controller    
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
            self._state["controller_on"] = time_since_read < num_wait_sec
            if poses == {}:
                continue

            # Determine Control Pipeline #
            toggled = self._state["movement_enabled"] != buttons[self.controller_id.upper() + "G"]
            self.update_sensor = self.update_sensor or buttons[self.controller_id.upper() + "G"]
            self.reset_orientation = self.reset_orientation or buttons[self.controller_id.upper() + "J"]
            self.reset_origin = self.reset_origin or toggled

            # Save Info #
            self._state["poses"] = poses
            self._state["buttons"] = buttons
            self._state["movement_enabled"] = buttons[self.controller_id.upper() + "G"]
            self._state["controller_on"] = True
            last_read_time = time.time()

            # Update Definition Of "Forward" #
            stop_updating = self._state["buttons"][self.controller_id.upper() + "J"] or self._state["movement_enabled"]
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

    def _process_reading(self):
        # the rotation matrix is a 4x4 matrix
        # need to first convert into quaternion by taking the upper left 3x3 matrix (rmat_to_quat)
        # the translational values are the last column (rightmost column)
        # take these values as the position values x, y, z
        rot_mat = np.asarray(self._state["poses"][self.controller_id])
        rot_mat = self.global_to_env_mat @ self.vr_to_global_mat @ rot_mat
        vr_pos = self.spatial_coeff * rot_mat[:3, 3] 
        vr_quat = rmat_to_quat(rot_mat[:3, :3])
        vr_gripper = self._state["buttons"]["rightTrig" if self.controller_id == "r" else "leftTrig"][0]

        self.vr_state = {"pos": vr_pos, "quat": vr_quat, "gripper": vr_gripper}

    # Try position control instead of velocity for now
    # need to have the IK for UR10e
    # def _limit_velocity(self, lin_vel, rot_vel, gripper_vel):
    #     """Scales down the linear and angular magnitudes of the action"""
    #     # basically: if the velocity exceeds the max, it turns it into normalized value
    #     # similar to unit vectors, works by multiplying the input vector by the quotient of the max vector/magnitude of input vector
    #     # otherwise, the input vector is not modified (magnitude is < max velocity)
    #     lin_vel_norm = np.linalg.norm(lin_vel)
    #     rot_vel_norm = np.linalg.norm(rot_vel)
    #     gripper_vel_norm = np.linalg.norm(gripper_vel)
    #     if lin_vel_norm > self.max_lin_vel:
    #         lin_vel = lin_vel * self.max_lin_vel / lin_vel_norm
    #     if rot_vel_norm > self.max_rot_vel:
    #         rot_vel = rot_vel * self.max_rot_vel / rot_vel_norm
    #     if gripper_vel_norm > self.max_gripper_vel:
    #         gripper_vel = gripper_vel * self.max_gripper_vel / gripper_vel_norm
    #     return lin_vel, rot_vel, gripper_vel

    def _calculate_action(self, state_dict, include_info=False):
        # Read Sensor #
        if self.update_sensor:
            self._process_reading()
            self.update_sensor = False

        # Read Observation
        robot_pos = np.array(state_dict["cartesian_position"][:3]) # first 3 values are x, y, z
        robot_euler = state_dict["cartesian_position"][3:] # last 3 vectors are xy, xz, yz
        robot_quat = euler_to_quat(robot_euler)
        robot_gripper = state_dict["gripper_position"]

        # Reset Origin On Release #
        if self.reset_origin:
            self.robot_origin = {"pos": robot_pos, "quat": robot_quat}
            self.vr_origin = {"pos": self.vr_state["pos"], "quat": self.vr_state["quat"]}
            self.reset_origin = False

        # Calculate Positional Action #
        robot_pos_offset = robot_pos - self.robot_origin["pos"]
        target_pos_offset = self.vr_state["pos"] - self.vr_origin["pos"]
        pos_action = target_pos_offset - robot_pos_offset

        # Calculate Euler Action #
        robot_quat_offset = quat_diff(robot_quat, self.robot_origin["quat"])
        target_quat_offset = quat_diff(self.vr_state["quat"], self.vr_origin["quat"])
        quat_action = quat_diff(target_quat_offset, robot_quat_offset)
        euler_action = quat_to_euler(quat_action)

        # Calculate Gripper Action #
        # the vr_state and robot_gripper are both normalized to 0-1
        # activate the gripper if it passes the threshold (0.5)
        gripper_action = (self.vr_state["gripper"]) - robot_gripper

        # # Calculate Desired Pose #
        target_pos = pos_action + robot_pos
        target_euler = add_angles(euler_action, robot_euler)
        target_cartesian = np.concatenate([target_pos, target_euler])
        # target_gripper = self.vr_state["gripper"]

        # # Scale Appropriately #
        # pos_action *= self.pos_action_gain
        # euler_action *= self.rot_action_gain
        # gripper_action *= self.gripper_action_gain
        # lin_vel, rot_vel, gripper_vel = self._limit_velocity(pos_action, euler_action, gripper_action)
        
        # desired pose
        # this is absolute/direct pose
        target_cartesian = np.concatenate([target_pos, target_euler])
        target_gripper = self.vr_state["gripper"]
        
        # scale the pose
        # may need to modify these values for ur10e
        target_pos *= self.pos_action_gain
        target_euler *= self.rot_action_gain
        # gripper_action *= self.gripper_action_gain
        # decided not to scale gripper action since it is already between 0 and 1

        # calculate delta action
        action = np.concatenate([target_pos, target_euler, [gripper_action]])

        # Prepare Return Values #
        # these are the values obtained from the oculus reader; target_gripper is the controller gripper state
        # the actual executed actions are stored in action
        info_dict = {"target_cartesian_position": target_cartesian, "target_gripper_position": target_gripper}
        # action = np.concatenate([lin_vel, rot_vel, [gripper_vel]])
        # action = action.clip(-1, 1) # limits values to -1, 1

        # Return #
        if include_info:
            return action, info_dict
        else:
            return action

    # method is not in use right now, but can call it to check the controller status
    def get_info(self):
        return {
            "success": self._state["buttons"]["A"] if self.controller_id == 'r' else self._state["buttons"]["X"],
            "failure": self._state["buttons"]["B"] if self.controller_id == 'r' else self._state["buttons"]["Y"],
            "movement_enabled": self._state["movement_enabled"],
            "controller_on": self._state["controller_on"],
        }

    def get_next_action(self, obs_dict, include_info=False):
        if self._state["poses"] == {}:
            action = np.zeros(7)
            if include_info:
                return action, {}
            else:
                return action # this is the delta (7 values)
        return self._calculate_action(obs_dict, include_info=include_info)


# In[ ]:


# # initialize the oculus reader
oculus_reader = OculusReader() # initialize oculusreader object
# controller_id = "r" # right controller; switch to "l" if using left controller

# # read controller
# last_read_time = time.time()
# num_wait_sec = 5
# # regulate read frequency #
# hz = 50
# time.sleep(1/hz)
# time_since_read = time.time() - last_read_time
# poses, buttons = oculus_reader.get_transformations_and_buttons()


# In[ ]:


# function to store the robot state in a obs_dict dictionary
def create_obs_dict(rtde_r, gripper):
    # return a dictionary with keys 'cartesian_position' and 'gripper_position'
    # where the cartesian position is the current tcp pose (x, y, z, rx, ry, rz)
    # and the gripper_position is a normalized value for the gripper status (original values are 0 to 255, we normalize to result in a range of 0 to 1)
    # octo takes the action with 1 as open and 0 as closed, but the gripper takes 255 as closed and 0 as open, so we need to flip by subtracting from 1
    
    obs_dict = {
        'cartesian_position': rtde_r.getActualTCPPose(),
        'gripper_position': round(1 - gripper.get_current_position()/255)
    }

    return obs_dict


# In[ ]:


# crude storage of robot pose in an array, which will be written to csv file
# store oculus reader output as well
# also store observation images in a directory:

# initialize robot pose array
# poses = [] # will store [x, y, z, rx - roll, ry - pitch, rz - yaw, GRIPPER]
# oculusPoses = [] # will store the poses from oculusreader
# oculusButtons = [] # will store the buttons from oculusreader

# # save observations as image strips later
# cameraObs = [] # for the default primary pointgrey camera
# cameraObsWrist = [] # for the realsense wrist camera (optional)

# initialize constants for naming
current_time = datetime.datetime.now()

nameString = f'outputs/run_{current_time.year}-{current_time.month}-{current_time.day}_{current_time.hour}-{current_time.minute}-{current_time.second}/'

# create csv files
poses = open(nameString + 'robotPose.csv', 'w', newline='') # store the poses [x, y, z, rx, ry, rz]
poseWriter = csv.writer(poses)

oculusPoses = open(nameString + 'oculusPose.csv', 'w', newline='') # store poses from oculusreader
oculusPoseWriter = csv.writer(oculusPoses)

oculusButtons = open(nameString + 'oculusButtons.csv', 'w', newline='') # store button state from oculusreader
oculusButtonWriter = csv.writer(oculusButtons)

# paths to store image observations
primary_cameraPath = nameString + 'cameraPrimary/'
wrist_cameraPath = nameString + 'cameraWrist/'

os.makedirs(primary_cameraPath, exist_ok=True)
os.makedirs(wrist_cameraPath, exist_ok=True)


# In[ ]:


# TO DO:
    # test on robot
    # for documented csv values, maybe also record the timestamp
    # look into IK for velocity-based ur10e control
    
count = 0 # for indexing and naming

# create vrpolicy object
oculus_policy = VRPolicy(right_controller = True) # change to false if using left controller

try:
    while True:
        # inside of this while loop, continuously grab actions calculated from the oculus
        # terminates teleop process if the user presses ESC on the cv2 window
        
        # if escape key is pressed on frame window
        if cv2.waitKey(1) % 0xFF == 27:
            # run visualize actions
            # visualizeActions(totalActions, images)
            # flag = True
            cv2.destroyAllWindows()
            # close the pointgrey camera
            print('Closing camera and RTDE.')

            pipeline.stop()
            pipeline_robot.stop()
            cam.EndAcquisition()
            # deinitialize camera
            cam.DeInit()
            del cam
            camera_list.Clear()
            system.ReleaseInstance()
            rtde_c.stopScript()
            break
        
        # recording robot poses data:
        # get the current tcp pose (6 values) and append the gripper value
        # the gripper position is from 0 to 255, where 0 is open and 255 is closed (source: https://sdurobotics.gitlab.io/ur_rtde/_static/robotiq_gripper.py)
        # we want to take range btwn 0 and 1; just divide by 255 to normalize
        # to align with octo, 1 is open and 0 is closed so we subtract from 255 and round all values to be 0 or 1
        # poses.append(rtde_r.getActualTCPPose().append(round(255 - gripper.get_current_position()/255)))
        
        # record oculusreader poses and buttons
        # last_read_time = time.time()
        oc_pose, oc_button = oculus_reader.get_transformations_and_buttons()
        # oculusPoses.append(poses)
        # oculusButtons.append(buttons)
        
        # record states and observations in csv files
        poseWriter.writerow(rtde_r.getActualTCPPose().append(round(255 - gripper.get_current_position()/255)))

        oculusPoses.writerow(oc_pose)
        
        oculusButtons.writerow(oc_button)
        
        # record camera observations
        cv2.imwrite(f'{primary_cameraPath}primary_{count}.png', primary_img)
        cv2.imwrite(f'{wrist_cameraPath}wrist_{count}.png', secondpov_frame)
        # cameraObs.append(primary_img[:, :, ::-1]) # convert from bgr to rgb
        # cameraObsWrist.append(secondpov_frame[:, :, ::-1]) # convert from bgr to rgb
        
        # release camera image
        frames.Release()
        
        # TO DO:
        # CALCULATE ROBOT ACTION
        # create observation dictionary
        obs_dict = create_obs_dict(rtde_r, gripper)
        
        # run get_next_action to calculate robot action based on oculus observation
        robot_action, info = oculus_policy.get_next_action(obs_dict=obs_dict, include_info=True)
        # robot_action contains [x, y, z, rx, ry, rz, gripper (0-1)]
        # info contains 
        
        # APPLY ROBOT ACTION
        print("Executing action: ", robot_action)
        tcp_pose = robot_action[:6].tolist() # first 6 values are the action
        gripper_position_val = robot_action[-1] # last value is the robot gripper from 0 to 1, where 0 is closed and 1 is open
        
        # robot pose update
        rtde_c.moveL(tcp_pose) # moveL to target position
        print("Movement complete. Proceeding to next action.")
        
        # gripper control
        if (gripper_position_val <= 0.5): # close the gripper
            gripper.move_and_wait_for_pos(255, 255, 255)
        else: # open the gripper
            gripper.move_and_wait_for_pos(0, 255, 255)
        
        # UPDATE OBSERVATION CAMERAS AND OTHER VARIABLES
        # update cameras
        # first camera:
        firstpov_frames = pipeline.wait_for_frames()
        color_frame = firstpov_frames.get_color_frame()
        depth_frame = firstpov_frames.get_depth_frame()

        # second camera:
        secondpov_frames = pipeline_robot.wait_for_frames()
        secondpov_color_frame = secondpov_frames.get_color_frame()
        secondpov_depth_frame = secondpov_frames.get_depth_frame()

        if not depth_frame or not color_frame:
            continue                        
        firstpov_frame = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        if not secondpov_color_frame or not secondpov_depth_frame:
            continue
        secondpov_frame = np.asanyarray(secondpov_color_frame.get_data())

        # reshape image frames to be consistent with octo data
        firstpov_frame = cv2.resize(np.array(firstpov_frame),(256,256))
        secondpov_frame = cv2.resize(np.array(secondpov_frame),(256,256))
        
        # get frames from gige camera
        frames = cam.GetNextImage()
        # make sure all frames are complete; not corrupted
        while True:
            if(frames.IsIncomplete()):
                print (f"Image incomplete with image status {frames.GetImageStatus()}")
                frames.Release()
                frames = cam.GetNextImage()
            else: break
        
        cv2_img = frames.GetData().reshape(height, width, 1)
        cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_BAYER_RG2RGB)
        primary_img = cv2.resize(np.array(cv2_img),(256,256))

        # show all camera views stacked together horizontally
        stacked_viewframes = np.hstack((cv2.flip(firstpov_frame, 1), secondpov_frame, primary_img))
        cv2.imshow("Realsense", stacked_viewframes)
        
        count +=1  # update counter index
              
except KeyboardInterrupt:
    print("Control Interrupted!")
    rtde_c.stopScript()
    
# stop all processes and close files
# close csv files
poses.close()
oculusPoses.close()
oculusButtons.close()

cv2.destroyAllWindows()
# close the pointgrey camera
print('Closing camera and RTDE.')
# close realsense cameras
pipeline.stop()
pipeline_robot.stop()
cam.EndAcquisition()
# deinitialize camera
cam.DeInit()
del cam
camera_list.Clear()
system.ReleaseInstance()
rtde_c.stopScript()

