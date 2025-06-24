# UR10e Teleoperation & Data Collection with Oculus Quest

This project enables teleoperation of a UR10e robotic arm using an Oculus Quest headset. It supports real-time control and data recording for a variety of manipulation tasks.

---

## 📁 Directory Structure

All scripts and utilities are organized to support:
- **teleop_controls** oculus reader repo and misc utils
- **ur10e_tele** working folder
- **camera_utils and trajectory_utils** folder for data saving code

---

## 🚀 Quick Start

### Step 1: Configure Task Parameters

Edit the following constants in  
`ur10e_tele/collect_URrobot_trajectory_test.py`:

```python
task = "place the pink stuffed animal in the box"  # Change this as needed
task_folder = "stuffed_toy"                        # 'stuffed_toy' or 'pills'
control_hz = 15                                    # Control frequency
save_hz = 5                                        # Save frequency
eps_horizon = 5                                    # Episode horizon
right_controller = True                            # Set to False if using left hand
save_data = True                                   # Whether to save data
```

For demo purposes set save_data to true and control frequency to 500hz for minimal latency. (code can't actually run at 500hz without a realtime kernel but setting it at 500hz will make the code run as fast as possible)

For saving data keep control_hz at 15 is recommneded. Save_hz should also be a multiple of your control frequency.
---

### Step 2: Run the Data Collection Script

From the **project root**, launch the script:

```bash
python ./ur10e_tele/collect_URrobot_trajectory_test.py
```

The program will:
- Start listening for VR controller input
- Record joint states, images, and metadata
- Save each session under the `outputs/{task_folder}/session_*` directory

After each episode:
- Press **A** for a successful trial
- Press **B** for a failed trial  
The program exits automatically after the desired number of episodes or if the **ESC** key is pressed.


If **ESC** is pressed or the program crashes, remove the last potentially corrupted episode and adjust your eps_horizon value to complete the remaining data collection.


---

### Step 3: Validate Collected Data

Use the notebook below to inspect and validate recorded trajectories:

```bash
ur10e_tele/trajectory_utils/trajectory_reader_rtde.ipynb
```

---

### Step 4: Review Outputs

All data will be stored in:

```
outputs/{task_folder}/session_*/
```
---

## Notes

- Ensure the robot is properly connected and the Oculus Quest controllers are active before starting.
    - usb connection need to be accepted 
    - working bountries need to be set in the oculus headset 
    - run teleop_controls/oculus_reader/oculus_reader/reader.py to check that oculus_reader is detected
- Refer to demo video for vr_control logic. 
- Common bugs (and solutions):
    - Strange movement (i.e. robot jerkiness/very small movements) is usually caused by oculus setup issues
        - Click on the Meta button to bring up the taskbar inside the oculus, then navigate to Settings --> Environment setup and click on "Create New Boundary"
        - Create an automatic new boundary (you will hear a sound cue) and close the Settings, then take off the headset and place it down
        - Position the controller above the oculus and HOLD the Meta button for ~1-2 seconds to recenter your virtual origin
    - Segmentation fault message in terminal:
        - Usually you can simply re-run the program; this sometimes happens when the previous script on the robot has not been terminated properly
- Oculus setup notes:
    - Connect the oculus to the extension cable
    - Press the Meta button to bring up the menu, then click on the notifications (bell icon)
    - If a USB message appears, click on it and accept the prompt
        - If there is no notification, disconnect and reconnect the cable
    - To check that the USB connection is properly enabled, enter `adb devices` and make sure the device list is: 1) not empty and 2) there is no message about permissions being unavailable
    - Create a new boundary where you will be placing the oculus and recenter the origin by holding the Meta button (you can follow instructions on Meta for this, or see above note "Common bugs")
    - Run `ur10e_tele/collect_URrobot_trajectory_test.py` to begin teleop script!

