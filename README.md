# Vibro Tactile Toolbox

## How to Run

### Step-by-Step Instructions

1. **Start rosmaster**:
   - Open a new tmux window named `roscore` and run:
     ```sh
     tmux new -t roscore
     roscore
     ```

2. **Start FT Sensor Publisher**:
   - Open a second tmux window and connect to `yk-god`:
     ```sh
     ssh yk-god
     rosrun fts_serial fts_serial_node __ns:="yk_builder"
     ```

3. **Start Dummy Screen** (if screen is not connected):
   - In the same tmux window, run:
     ```sh
     ssh yk-god
     start_dummy_screen
     ```

4. **Start Realsense Wrist Camera**:
   - Connect to `yk-god` with X11 forwarding:
     ```sh
     ssh -X yk-god
     export DISPLAY=:0
     roslaunch realsense2_camera rs_aligned_depth.launch
     ```

5. **Start Femto Bolt**:
   - In the same tmux window, run:
     ```sh
     ssh -X yk-god
     export DISPLAY=:0
     roslaunch vibro_tactile_toolbox femto_bolt.launch
     ```

6. **Start Femto Bolt Cropped**:
   - Run the following command:
     ```sh
     ssh yk-god
     rosrun vibro_tactile_toolbox image_cropped_republisher.py
     ```

7. **Start Microphones**:
   - Run the following command:
     ```sh
     ssh yk-god
     rosrun sounddevice_ros sounddevice_ros_publisher_node.py -d 3 -c 3
     ```

8. **Start Terminators**:
   - Run the following command:
     ```sh
     roslaunch vibro_tactile_toolbox terminators.launch
     ```

9. **Start Robot**:
   - In a new tmux window, run:
     ```sh
     roslaunch testbed_utils lego_moveit_yk.launch namespace:=yk_creator
     ```
   - If prompted, enable the robot:
     ```sh
     rosservice call /yk_creator/robot_enable
     ```

10. **Start Robot Pose Publisher**:
    - In another tmux window, run:
      ```sh
      rosrun vibro_tactile_toolbox pose_stamped_publisher.py -n yk_creator -f 100
      ```

11. **Start the Lego Detector in a Docker Container**:
    - Navigate to the vibro_tactile_toolbox directory and run:
      ```sh
      roscd vibro_tactile_toolbox
      cd docker
      ./run -i noetic_vibro_tactile_toolbox -c lego_detector -g
      source /ros1_ws/devel/setup.bash
      cd /home/repos/ros1_ws/src/kevin/vibro_tactile_toolbox/
      git pull
      python3 scripts/lego_detector.py
      ```
    - To test that it's running:
      ```sh
      python3 test_lego_detector.py -t '/side_camera/color/image_cropped'
      ```

12. **Start the FTS Detector**:
    - Navigate to the vibro_tactile_toolbox directory and run:
      ```sh
      roscd vibro_tactile_toolbox
      python3 scripts/fts_detector.py
      ```
    - To test that it's running:
      ```sh
      python3 test_fts_detector.py -t "fts"
      ```

### Revised Procedure

1. **Start rosmaster**:
   - Open a new tmux session named `roscore` on `mfi-twin`:
     ```sh
     tmux new -t roscore
     roscore
     ```

2. **Start RTC Drivers**:
   - In a new tmux session named `RTC-drivers` on `mfi-twin`, run:
     ```sh
     cd ~
     bash launch_RTC_drivers.sh
     ```

3. **Start Yaskawa Drivers**:
   - Open a new tmux session named `yk-creator` on `mfi-twin`:
     ```sh
     tmux new -t yk-creator
     ```
   - Start MoveIt in a new tmux window (don't use tmux if you want to use RViz):
     ```sh
     tmux new-window -n moveit
     roslaunch testbed_utils lego_moveit_yk.launch namespace:=yk_creator
     ```
   - Start Pose Stamped Publisher in another new tmux window:
     ```sh
     tmux new-window -n pose_stamped_publisher
     rosrun vibro_tactile_toolbox pose_stamped_publisher.py -n yk_creator -f 100
     ```

4. **Start VTT Core Drivers**:
   - Open a new tmux session named `VTT-core` on `mfi-twin` and run:
     ```sh
     roscd vibro_tactile_toolbox
     bash launch_VTT_core.sh
     ```

5. **Launch the Desired Task Script**:
   - Example to collect tactile data:
     ```sh
     roslaunch vibro_tactile_toolbox collect_tactile_data.launch
     ```
   - Example to go home:
     ```sh
     roslaunch vibro_tactile_toolbox go_home_skill.launch
     ```


### NIST Data Collection Instructions
1. **Start rosmaster**:
   - Open a new tmux session named `roscore` on `mfi-twin`:
     ```sh
     tmux new -t roscore
     roscore
     ```

2. **Start Yaskawa Drivers**:
   - Open a new tmux session named `yk-builder` on `mfi-twin`:
     ```sh
     tmux new -t yk-builder
     ```
   - Start MoveIt in a new tmux window (don't use tmux if you want to use RViz):
     ```sh
     tmux new-window -n moveit
     roslaunch testbed_utils lego_moveit_yk.launch namespace:=yk_builder
     ```

3. **Start FTS Sensor on Yk-god**:
    - Open a terminal window and connect to `yk-god`:
     ```sh
     ssh yk-god
     tmux a
     rosrun fts_serial multi_fts_serial_node
     ```
    
4. **Start Gripper, Camera, and Audio Drivers on r221-lambda**:
    - Open a terminal window and connect to `r221-lambda`:
     ```sh
     ssh r221-lambda
     cd repos/kevin/vibro_tactile_toolbox/docker
     docker compose up
     ```

5. **Start Outcome Detection, Terminators, and Pose Stamped Publisher on MFI Twin**:
    - Open a new tmux session named `nist-drivers` on `mfi-twin`:
     ```sh
     tmux new -t nist-drivers
     ```
    - Start mfi_twin_nist in a new tmux window:
     ```sh
     tmux new-window -n nist-launch
     roslaunch vibro_tactile_toolbox mfi_twin_nist.launch
     ```
6. **Start Data Collection Script on MFI Twin**:
    - Open a new tmux session named `nist_data_collection` on `mfi-twin`:
     ```sh
     tmux new -t nist_data_collection
     ```
    - Start mfi_twin_nist in a new tmux window:
     ```sh
     tmux new-window -n nist_data_collection
     roslaunch vibro_tactile_toolbox collect_nist_tactile_data.launch
     ```


### Lego Data Collection Instructions
1. **Start rosmaster**:
   - Open a new tmux session named `roscore` on `mfi-twin`:
     ```sh
     tmux new -t roscore
     roscore
     ```

2. **Start Yaskawa Drivers**:
   - Open a new tmux session named `yk-creator` on `mfi-twin`:
     ```sh
     tmux new -t yk-creator
     ```
   - Start MoveIt in a new tmux window (don't use tmux if you want to use RViz):
     ```sh
     tmux new-window -n moveit
     roslaunch testbed_utils lego_moveit_yk.launch namespace:=yk_creator
     ```

3. **Start Wrist, Side Cameras, and Audio on Yk-god**:
    - Open a terminal window and connect to `yk-god`:
     ```sh
     ssh yk-god
     roslaunch vibro_tactile_toolbox yk_god_lego.launch
     ```

4. **Start Outcome Detection, Terminators, and Pose Stamped Publisher on MFI Twin**:
    - Open a new tmux session named `lego-drivers` on `mfi-twin`:
     ```sh
     tmux new -t lego-drivers
     ```
    - Start mfi_twin_lego in a new tmux window:
     ```sh
     tmux new-window -n lego-launch
     roslaunch vibro_tactile_toolbox mfi_twin_lego.launch
     ```
5. **Start Data Collection Script on MFI Twin**:
    - Open a new tmux session named `lego_data_collection` on `mfi-twin`:
     ```sh
     tmux new -t lego_data_collection
     ```
    - Start mfi_twin_lego in a new tmux window:
     ```sh
     tmux new-window -n lego_data_collection
     roslaunch vibro_tactile_toolbox collect_lego_tactile_data.launch
     ```