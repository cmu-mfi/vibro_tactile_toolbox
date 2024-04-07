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

10. Start Robot Pose Publisher (Some Tmux window)
rosrun vibro_tactile_toolbox pose_stamped_publisher.py -n yk_creator -f 100