import os
import random
from collections import defaultdict

# from camera_utils.camera_readers.zed_camera import gather_zed_cameras
# from camera_utils.info import get_camera_type
from teleop_controls.misc.subprocess_utils import run_threaded_command

import PySpin
import pyrealsense2 as rs
import cv2
import numpy as np
import datetime


class MultiCameraWrapper:
    def __init__(self):
        
        # recording folder declaration
        # current_time = datetime.datetime.now()
        # nameString = f'outputs/run_{current_time.year}-{current_time.month}-{current_time.day}_{current_time.hour}-{current_time.minute}/'
        # os.makedirs(nameString, exist_ok=True)
        # self.recording_folderpath = nameString
        
        print("Getting cameras set up...")

        # open the realsense cameras
        # first camera
        # first camera displays the robot's wrist camera view for better depth perception
        self.pipeline_robot = rs.pipeline()
        self.config_robot = rs.config()
        self.config_robot.enable_device('218622276856')
        print(f"Device: Intel Realsense D405 - Robot Wrist Camera")
        self.config_robot.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
        # self.config_robot.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, )
        
        # pointgrey camera
        # primary view - pointgrey 3rd person camera
        # retrieve camera
        serial = '13440209'

        self.system = PySpin.System.GetInstance()
        self.camera_list = self.system.GetCameras()
        num_cams = self.camera_list.GetSize()

        if num_cams == 0:
            self.camera_list.Clear()
            print('No cameras found.')

        # fetch camera
        self.cam = self.camera_list.GetBySerial(serial)

        # Open Cameras #
        # zed_cameras = gather_zed_cameras()
        # self.camera_dict = {cam.serial_number: cam for cam in zed_cameras}

        # Set Correct Parameters #
        # for cam_id in self.camera_dict.keys():
        #     cam_type = get_camera_type(cam_id)
        #     curr_cam_kwargs = camera_kwargs.get(cam_type, {})
        #     self.camera_dict[cam_id].set_reading_parameters(**curr_cam_kwargs)

        # Launch Cameras #
        run_threaded_command(self.set_trajectory_mode())

    ### Calibration Functions ###
    # def get_camera(self, camera_id):
    #     return self.camera_dict[camera_id]

    # def enable_advanced_calibration(self):
    #     for cam in self.camera_dict.values():
    #         cam.enable_advanced_calibration()

    # def disable_advanced_calibration(self):
    #     for cam in self.camera_dict.values():
    #         cam.disable_advanced_calibration()

    # def set_calibration_mode(self, cam_id):
    #     # If High Res Calibration, Only One Can Run #
    #     close_all = any([cam.high_res_calibration for cam in self.camera_dict.values()])

    #     if close_all:
    #         for curr_cam_id in self.camera_dict:
    #             if curr_cam_id != cam_id:
    #                 self.camera_dict[curr_cam_id].disable_camera()

    #     self.camera_dict[cam_id].set_calibration_mode()

    def set_trajectory_mode(self):
        # start all cameras
        # realsense camera:
        self.pipeline_robot.start(self.config_robot)
        
        # pointgrey camera:
        self.cam.Init()
        nodemap = self.cam.GetNodeMap()
        self.cam.PixelFormat.SetValue(PySpin.PixelFormat_BayerRG8)
        print (f"Pixel format set to: {self.cam.PixelFormat.GetCurrentEntry().GetSymbolic()}")

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
        
        # put into acquisition mode
        self.cam.BeginAcquisition()
        
        # If High Res Calibration, Close All #
        # close_all = any(
        #     [cam.high_res_calibration and cam.current_mode == "calibration" for cam in self.camera_dict.values()]
        # )

        # if close_all:
        #     for cam in self.camera_dict.values():
        #         cam.disable_camera()

        # Put All Cameras In Trajectory Mode #
        # for cam in self.camera_dict.values():
        #     cam.set_trajectory_mode()

    ### Data Storing Functions ###
    # def start_recording(self):
    #     # paths to store image observations
    #     self.primary_cameraPath = self.recording_folderpath + 'cameraPrimary/'
    #     self.wrist_cameraPath = self.recording_folderpath + 'cameraWrist/'

    #     os.makedirs(self.primary_cameraPath, exist_ok=True)
    #     os.makedirs(self.wrist_cameraPath, exist_ok=True)
        
    #     return self.primary_cameraPath, self.wrist_cameraPath
        
        # for cam in self.camera_dict.values():
        #     filepath = os.path.join(subdir, cam.serial_number + ".svo")
        #     cam.start_recording(filepath)

    def stop_recording(self):
        # for cam in self.camera_dict.values():
        #     cam.stop_recording()
        # disable all cameras and stop recording
        self.pipeline_robot.stop()
        self.cam.EndAcquisition()
        # deinitialize camera
        self.cam.DeInit()
        del self.cam
        self.camera_list.Clear()
        self.system.ReleaseInstance()

    ### Basic Camera Functions ###
    def read_cameras(self):
        # fetch new camera frames
        # first camera:
        secondpov_frames = self.pipeline_robot.wait_for_frames()
        secondpov_color_frame = secondpov_frames.get_color_frame()
        # secondpov_depth_frame = secondpov_frames.get_depth_frame()
                                       
        secondpov_frame = np.asanyarray(secondpov_color_frame.get_data())
        # depth_image = np.asanyarray(secondpov_depth_frame.get_data())

        # reshape image frames to (480, 640, 3)
        secondpov_frame = cv2.resize(np.array(secondpov_frame),(480,640))
        
        # get frames from gige camera
        frames = self.cam.GetNextImage()
        # make sure all frames are complete; not corrupted
        while True:
            if(frames.IsIncomplete()):
                print (f"Image incomplete with image status {frames.GetImageStatus()}")
                frames.Release()
                frames = self.cam.GetNextImage()
            else: break
        
        cv2_img = frames.GetData().reshape(self.cam.Height(), self.cam.Width(), 1)
        cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_BAYER_RG2RGB)
        primary_img = cv2.resize(np.array(cv2_img),(480,640))
        
        # clear the frames each time to prevent filling buffer
        frames.Release()
        
        return secondpov_frame, primary_img
        # full_obs_dict = defaultdict(dict)
        # full_timestamp_dict = {}

        # # Read Cameras In Randomized Order #
        # all_cam_ids = list(self.camera_dict.keys())
        # random.shuffle(all_cam_ids)

        # for cam_id in all_cam_ids:
        #     if not self.camera_dict[cam_id].is_running():
        #         continue
        #     data_dict, timestamp_dict = self.camera_dict[cam_id].read_camera()

        #     for key in data_dict:
        #         full_obs_dict[key].update(data_dict[key])
        #     full_timestamp_dict.update(timestamp_dict)

        # return full_obs_dict, full_timestamp_dict

    # def disable_cameras(self):
    #     for camera in self.camera_dict.values():
    #         camera.disable_camera()
