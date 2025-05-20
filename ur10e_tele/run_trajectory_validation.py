import sys
import os
import csv
import numpy as np
import time

# Get the absolute path of the project root
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.append(project_root)

from ur10e_tele.RTDE_RW_test_collect import RobotAction
from teleop_controls.misc.transformations import quat_to_euler
from teleop_controls.misc.time import time_ms


# make the robot env
env = RobotAction(control_hz=5)

env.reset()

print("Ready for validation!")

# grab the csv file for robot states
robot_state = open("/home/demoaccount/Data/demoaccount2/teleop_data_collect/ur10e_tele/trajectory_utils/results/session_1/episode_0/action.csv")

# skip the heading
heading = next(robot_state)

# reader object
filereader = csv.reader(robot_state)

# count = 0

# iterate over each row:
for row in filereader:
    # if count%3 == 0:
        # Regularize Control Frequency , it subtracts from compute time to provide consistant frequency
        control_timestamps = time_ms()

        # read tcp pose and convert to euler
        # robot_coor = np.array([float(x) for x in row[6:9]])
        # robot_quat = np.array([float(x) for x in row[9:13]])
        robot_coor = np.array([float(x) for x in row[:3]])
        robot_euler = np.array([float(x) for x in row[3:6]])
        # robot_euler = quat_to_euler(robot_quat)
        
        # flip the recorded gripper state
            # robot action update accepts 0 as open and 1 as closed (opposite to recorded data)
        gripper_action = np.array(int(float(row[-1])))
        
        # concat into robot_action
        robot_action = np.concatenate([robot_coor, robot_euler, [gripper_action]])
        
        # action blocking is the last value
            # flip this as well bc the recorded state is action_blocked
            # if action_blocked is false, then non_blocking is true, and vice versa
        # blocking = 1 - np.array(int(float(row[-1])))
        
        comp_time = time_ms() - control_timestamps
        sleep_left = (1000 / env.control_hz) - (comp_time)
        if sleep_left > 0:
            time.sleep(sleep_left/1000) # sleep in s

        # move the robot with send_action
        # env.send_pose(action=robot_action, non_blocking=True)
        env.send_action(action=robot_action, non_blocking=True)

    # count+=1

# close everything
robot_state.close()
env.close()