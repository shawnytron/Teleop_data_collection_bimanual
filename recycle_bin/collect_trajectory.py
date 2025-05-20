from controllers.oculus_controller import VRPolicy
from ur10e_tele.robot_env import RobotEnv
from ur10e_tele.trajectory_utils.misc import collect_trajectory

# Make the robot env
env = RobotEnv()
controller = VRPolicy()

print("Ready")
collect_trajectory(env, controller=controller)
