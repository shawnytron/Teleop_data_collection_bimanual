from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import robotiq_gripper
from teleop_controls.misc.time import time_ms
import time 
import math 
import numpy as np 
from teleop_controls.misc.transformations import axis_to_euler , euler_to_axis , add_poses, Euler2Axis_Pose
from teleop_controls.misc.subprocess_utils import run_threaded_command


class RobotAction:
    def __init__(self, robot_ip="192.168.0.110", acceleration=0.50, velocity=0.5, do_reset=True):
        """
        Initialize the RTDE control/receive interfaces and the gripper.
        """
        self.robot_ip = robot_ip
        self.acceleration = acceleration
        self.velocity = velocity
        self.pose = []
        self.gripper_current_state=False

        #robot control frequency test from slow to faster the 50hz 
        self.control_hz = 500

        # Parameters
        rtde_frequency = 500.0
        flags = RTDEControl.FLAG_VERBOSE | RTDEControl.FLAG_UPLOAD_SCRIPT
        ur_cap_port = 50002

        # ur_rtde realtime priorities
        rt_receive_priority = 90
        rt_control_priority = 85

        # Initialize ur-rtde recieve and control commands
        self.rtde_r = RTDEReceive(self.robot_ip, rtde_frequency, [], True, False, rt_receive_priority)
        self.rtde_c = RTDEControl(self.robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority)

        # Initialize the gripper
        print("Creating gripper...")
        self.gripper = robotiq_gripper.RobotiqGripper()
        print("Connecting to gripper...")
        self.gripper.connect(self.robot_ip, 63352)
        print("Activating gripper...")
        self.gripper.activate()
        time.sleep(1)  # Wait for the gripper to activate

        print("Begin updating internal robot state")
        run_threaded_command(self._update_actual_pose)

        #add a robot reset later on maybe? 
    # def get_state(self):#NEED TO IMPLEMENT TIMESTEP RECORDING 
    #         read_start = time_ms()
    #         state_dict, timestamp_dict = self._robot.get_robot_state()
    #         timestamp_dict["read_start"] = read_start
    #         timestamp_dict["read_end"] = time_ms()
    #         return state_dict, timestamp_dict
    
    def reset(self):
        self.move_to_start_position()
        
    def move_to_start_position(self,start_position = (
        math.radians(262.85),
        math.radians(-87.14),
        math.radians(111.61),
        math.radians(246.68),
        math.radians(-89.5),
        math.radians(-10.81)
        )):
        #move to start position 
        """
        robot_startposition = (
        math.radians(262),
        math.radians(-71),
        math.radians(119),
        math.radians(137),
        math.radians(-80),
        math.radians(-3)
        )
        """
        print("Moving robot to start position...")
        self.rtde_c.moveJ(start_position)
        print("Robot moved to start position.")
    

    ##implement a threading operation to constantly refresh obtaining robotstate from robot to ensure both observation and update action has freshst robot position
    def _update_actual_pose(self, hz=300): #this frequency can be gradually increased if ubuntu non-realtime kernal can handle it. 
        last_read_time = time.time()
        while True:
            # Regulate Read Frequency #
            time.sleep(1 / hz)
            # Read Controller
            self.pose = np.array(self.rtde_r.getActualTCPPose())

    def get_state(self):
        read_start = time_ms()
        timestamp_dict = {}
        timestamp_dict["read_start"] = read_start
        timestamp_dict["read_end"] = time_ms()
        return timestamp_dict
            
    def get_observation(self):
        """
        convert [x,y,z,wx,wy,wz] -> [x,y,z, eular angles]
        """
        state_dict= {}
        #get TCP pose (position+ then convert to euler angles)
        # Get the 6D TCP pose (x, y, z, rx, ry, rz)
        tcp_pose = self.pose
        robot_pos = tcp_pose[:3]
        robot_euler = axis_to_euler(tcp_pose[3:])
        tcp_pose_euler = np.concatenate([robot_pos, robot_euler])
        state_dict["cartesian_position"] = tcp_pose_euler
        #state_dict["cartesian_position"] = tcp_pose #pase axis_angle

        #need to add gripper action reading in the future as well
        # Determine the gripper's state
        if self.gripper.is_open():
            gripper_state = 0
        else:
            gripper_state = 1

        state_dict["gripper_position"] = gripper_state
        return state_dict
    
    def send_action(self,action,non_blocking=True,velocity=0.30,accel=0.20):
        #TODO: Add gripper action parsing
        pose_action = np.array(action)[:6]
        gripper_state = int(np.array(action)[-1])
        self.update_commands(pose_action,non_blocking,velocity=velocity,accel=accel)
       
        #toggle type control for gripper
        if gripper_state != self.gripper_current_state:
            gripper_control_value = int(gripper_state*255)
            self.send_gripper_command(gripper_control_value)


    def update_commands(self,new_action,non_blocking,velocity,accel): #blocking is set to false by default set in send_action
        """
        send command to robot using servoJ(x,y,z,wx,wy,wz)
        """
        #Parmeters
        dt = 1.0/self.control_hz
        lookahead_time = 0.1
        gain = 400

        #convert current_pose to euler
        current_pos = self.pose[:3]
        current_euler = axis_to_euler(self.pose[3:])
        current_euler_pose = np.concatenate([current_pos, current_euler])

        #now both rotation are in euler ready to be added then converted to axis angle to send to robot

        #swap axis for translation 
        
        target_euler= add_poses(new_action,current_euler_pose)
        target = Euler2Axis_Pose(target_euler).tolist()
        self.rtde_c.servoL(target,velocity, accel, dt, lookahead_time, gain)

        
    def send_gripper_command(self, gripper_value, speed=70, force=50):
        """
        Send a command to the gripper. and update internal state of gripper
        :param gripper_value: An integer value between 0 (open) and 255 (closed)
        :param speed: Speed for gripper motion
        :param force: Force for gripper motion
        """
        self.gripper_current_state = not self.gripper_current_state
        print(f"Sending gripper command: {gripper_value} (Speed: {speed}, Force: {force})")
        (self.gripper.move(gripper_value, speed, force))
        
    def close(self):
        """
        Disconnect the RTDE interfaces and the gripper.
        """
        print("Closing RTDE interfaces and disconnecting gripper...")
        self.rtde_c.disconnect()
        self.rtde_r.disconnect()
        self.gripper.disconnect()


