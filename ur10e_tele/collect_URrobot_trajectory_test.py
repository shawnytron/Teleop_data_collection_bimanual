import sys
import os
import datetime

# Get the absolute path of the project root
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.append(project_root)

from controllers.BooleanGripperCartesianAction import VRPolicy
# from ur10e_tele.RTDE_RW import RobotAction
from ur10e_tele.RTDE_RW_test_collect import RobotAction
from ur10e_tele.trajectory_utils.RTDE_UR_misc_Trial_Collection import collect_trajectory

# CONSTANTS
# task = "move pills from one plate to another plate" # change this as needed
    # move pills from one plate to another plate OR place the pink stuffed animal in the box
task = "place the pink stuffed animal in the box"
# task_folder = "pills" # pills OR stuffed_toy
task_folder = "stuffed_toy"
control_hz=500
save_hz=5
eps_horizon = 50
right_controller = True
save_data = False
nameString = ""


# Make the robot env
env = RobotAction(control_hz=control_hz)
controller = VRPolicy(right_controller=right_controller)

# initialize constants for data recording
# current_time = datetime.datetime.now()

if(save_data):
    # make parent directory if it does not exist
    os.makedirs(f'outputs/{task_folder}', exist_ok=True)

    # get the new session id
    outputs_list = os.listdir(f'outputs/{task_folder}')

    if outputs_list == None:
        new_session_id = 0
    else: new_session_id = len(outputs_list)

    # create new folder for session
    nameString = f'outputs/{task_folder}/session_{new_session_id}/'
    # nameString = f'outputs/session_{current_time.year}-{current_time.month}-{current_time.day}_{current_time.hour}-{current_time.minute}-{current_time.second}/'

    os.makedirs(nameString, exist_ok=True)


#get starting index if none are given then will start at 0
# starting_index = int(input("Enter the starting index ID (default 0): ") or 0)
# print(f"The starting index ID is: {starting_index}")

print("Ready")

try:
    # Call collect_trajectory and check for an empty return
    result = collect_trajectory(
        env, 
        controller=controller, 
        save_filepath=nameString, 
        task=task, 
        save_images=True, 
        save_data=save_data, 
        save_hz=save_hz, 
        eps_horizon=eps_horizon,
        control_hz=control_hz,
        right_controller=right_controller,
        # start_index=starting_index
    )
    
    # If the result is None (empty return), exit the program
    if result is None:
        print("collect_trajectory returned nothing, exiting the program.")
        sys.exit(0)  # Exit the program gracefully
        
except KeyboardInterrupt:
    env.close()
    print("Closing EVERYTHING!!!!")
    sys.exit(0)  # Ensure the program exits cleanly
