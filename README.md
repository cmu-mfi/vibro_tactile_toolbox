# vibro_tactile_toolbox

## How to run

1. Start Rosmaster (First Tmux window)
roscore

2. Start FT Sensor publisher (Second Tmux window)
ssh yk-god
rosrun fts_serial fts_serial_node

3. Start Dummy screen (if screen not connected)
ssh yk-god
start_dummy_screen

4. Start Realsense Wrist Camera
ssh -X yk-god
export DISPLAY=:0
roslaunch realsense2_camera rs_aligned_depth.launch 

5. Start Femto Bolt
ssh -X yk-god
export DISPLAY=:0
roslaunch vibro_tactile_toolbox femto_bolt.launch

6. Start Femto Bolt cropped
ssh yk-god
rosrun vibro_tactile_toolbox image_cropped_republisher.py

7. Start Microphones
ssh yk-god
rosrun sounddevice_ros sounddevice_ros_publisher_node.py -d 2 -c 2

8. Start Terminators
roslaunch vibro_tactile_toolbox terminators.launch

9. Start Robot (Some Tmux window)
roslaunch testbed_utils lego_moveit_yk.launch namespace:=yk_creator
# if prompted need to call robot enable 
# $ rosservice call /yk_creator/robot_enable

10. Start Robot Pose Publisher (Some Tmux window)
rosrun vibro_tactile_toolbox pose_stamped_publisher.py -n yk_creator -f 100

11. Start the Lego detector in a docker container.
roscd vibro_tactile_toolbox
cd docker
./run  -i noetic_vibro_tactile_toolbox -c lego_detector -g
source /ros1_ws/devel/setup.bash
cd /home/repos/ros1_ws/src/kevin/vibro_tactile_toolbox/
git pull
python3 scripts/lego_detector.py 

# to test that it's running
# $ python3 test_lego_detector.py -t '/side_camera/color/image_cropped'

12. Start the fts detector.
roscd vibro_tactile_toolbox
python3 scripts/fts_detector.py

# to test that it's running
# $ python3 test_fts_detector.py -t "fts"


# Revised:

1. Start rosmaster by starting roscore in a "roscore" tmux session. This should always be running on mfi-twin
- tmux new -t roscore
- roscore

2. Start the RTC drivers in a "RTC-drivers" tmux session on mfi-twin; this starts the fts, dummy screen, realsense, femtobolt, femtoboltcropped, audio
- cd ~
- bash launch_RTC_drivers.sh

3. Start the yaskawa drivers in a seperate "yk-creator" tmux session. This should ususally be running on mfi-twin
- tmux new -t yk-creator
# Don't put moveit in tmux if you want to use RVIZ
- tmux new-window -n moveit
- roslaunch testbed_utils lego_moveit_yk.launch namespace:=yk_creator
- tmux new-window -n pose_stamped_publisher
- rosrun vibro_tactile_toolbox pose_stamped_publisher.py -n yk_creator -f 100

4. Start the VTT core drivers (terminator, outcomer) in a "VTT-core" tmux session on mfi-twin
# This will change to a roslaunch file but bash is quick
- roscd vibro_tactile_toolbox
- bash launch_VTT_core


5. Launch the desired task script
- Ex: roslaunch vibro_tactile_toolbox collect_tactile_data.launch 
- Ex: roslaunch vibro_tactile_toolbox go_home_skill.launch













