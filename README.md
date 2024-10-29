# Vibro Tactile Toolbox

## Running Instructions

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

2. **Start FT Sensor Publisher**:
   - Open a second tmux window and connect to `yk-god`:
     ```sh
     ssh yk-god
     rosrun fts_serial multi_fts_serial_node
     ```

3. **Start Yaskawa Drivers**:
   - Open a new tmux session named `yk-creator` on `mfi-twin`:
     ```sh
     tmux new -t yk-creator
     ```
   - Start MoveIt in a new tmux window (don't use tmux if you want to use RViz):
     ```sh
     tmux new-window -n moveit
     roslaunch yk_launch moveit.launch namespace:=yk_creator
     ```

4. **Start Wrist, Side Cameras, and Audio on Yk-god**:
    - Open a terminal window and connect to `yk-god`:
     ```sh
     ssh yk-god
     roslaunch vibro_tactile_toolbox yk_god_lego.launch
     ```

5. **Start Outcome Detection, Terminators, and Pose Stamped Publisher on MFI Twin**:
    - Open a new tmux session named `lego-drivers` on `mfi-twin`:
     ```sh
     tmux new -t lego-drivers
     ```
    - Start mfi_twin_lego in a new tmux window:
     ```sh
     tmux new-window -n lego-launch
     roslaunch vibro_tactile_toolbox mfi_twin_lego.launch
     ```
6. **Start Data Collection Script on MFI Twin**:
    - Open a new tmux session named `lego_data_collection` on `mfi-twin`:
     ```sh
     tmux new -t lego_data_collection
     ```
    - Start mfi_twin_lego in a new tmux window:
     ```sh
     tmux new-window -n lego_data_collection
     roslaunch vibro_tactile_toolbox collect_lego_tactile_data.launch
     ```