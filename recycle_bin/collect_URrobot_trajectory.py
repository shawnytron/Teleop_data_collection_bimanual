import sys
import os

# Get the absolute path of the project root
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.append(project_root)

from controllers.euler import VRPolicy
# from ur10e_tele.RTDE_RW import RobotAction
from ur10e_tele.RTDE_RW import RobotAction
from ur10e_tele.trajectory_utils.RTDE_UR_misc_default import collect_trajectory

# Make the robot env
env = RobotAction()
controller = VRPolicy()

print("Ready")
collect_trajectory(env, controller=controller)
