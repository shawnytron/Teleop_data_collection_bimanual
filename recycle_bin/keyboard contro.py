import keyboard
import time
import rtde_control
import rtde_receive
import math 

class KeyboardControl:
    def __init__(self):
        # Initial position and orientation deltas
        self.delta = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Mapping keys to translation/rotation steps
        self.step = 0.01  # Movement step for translation and rotation
        self.rotation_step = 1.0  # Step size for rotation
        self.translation_keys = {'w': [0.01, 0.0, 0.0], 's': [-0.01, 0.0, 0.0], 
                                 'a': [0.0, 0.01, 0.0], 'd': [0.0, -0.01, 0.0], 
                                 'space': [0.0, 0.0, 0.01], 'left alt': [0.0, 0.0, -0.01]}
        self.rotation_keys = {'up': [self.rotation_step, 0.0, 0.0], 'down': [-self.rotation_step, 0.0, 0.0], 
                              'left': [0.0, self.rotation_step, 0.0], 'right': [0.0, -self.rotation_step, 0.0], 
                              'num 1': [0.0, 0.0, self.rotation_step], 'num 0': [0.0, 0.0, -self.rotation_step]}
    
    def update_delta(self):
        # Check for key presses and update the delta
        for key, trans in self.translation_keys.items():
            if keyboard.is_pressed(key):
                self.delta[0] += trans[0]  # x translation
                self.delta[1] += trans[1]  # y translation
                self.delta[2] += trans[2]  # z translation
        for key, rot in self.rotation_keys.items():
            if keyboard.is_pressed(key):
                self.delta[3] += rot[0]  # wx rotation
                self.delta[4] += rot[1]  # wy rotation
                self.delta[5] += rot[2]  # wz rotation

    def get_delta(self):
        return self.delta

    def reset_delta(self):
        self.delta = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

class RobotController:
    def __init__(self):
        self.rtde_c = rtde_control.RTDEControlInterface("192.168.0.110")
        self.rtde_r = rtde_receive.RTDEReceiveInterface("192.168.0.110")
        self.keyboard_control = KeyboardControl()

    def run(self):
        # Set the robot's starting position to the given joint angles
        robot_startposition = (
            math.radians(252),
            math.radians(-77),
            math.radians(132),
            math.radians(161),
            math.radians(-67),
            math.radians(-22)
        )
        self.rtde_c.moveJ(robot_startposition)

        # Main control loop
        try:
            while True:
                # Update the delta values based on key presses
                self.keyboard_control.update_delta()
                delta = self.keyboard_control.get_delta()

                # Apply translation and rotation deltas
                # Get current robot pose
                current_pose = self.rtde_r.getActualTCPPose()
                
                # Apply translation deltas
                current_pose[0] += delta[0]
                current_pose[1] += delta[1]
                current_pose[2] += delta[2]
                
                # Apply rotation deltas (this is a simple placeholder, you would need proper rotation handling)
                current_pose[3] += delta[3]
                current_pose[4] += delta[4]
                current_pose[5] += delta[5]

                # Move robot to new position
                self.rtde_c.moveL(current_pose, 0.25, 0.5, True)

                # Reset delta after applying to avoid accumulation
                self.keyboard_control.reset_delta()

                # Exit the loop if 'q' is pressed
                if keyboard.is_pressed('q'):
                    break

                time.sleep(0.1)  # A small delay to allow other processes to handle input
        finally:
            self.rtde_c.stopScript()
            print("Robot control script stopped.")

# Run the robot control scriptde
robot_controller = RobotController()
robot_controller.run()
