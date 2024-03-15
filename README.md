# vibro_tactile_toolbox

## How to run

FT Sensor publisher
rosrun fts_serial fts_serial_node

Realsense Wrist Camera
roslaunch realsense2_camera rs_aligned_depth.launch serial_no:=123622270997 camera:=wrist_camera

Microphones
rosrun sounddevice_ros sounddevice_ros_publisher_node.py -d 2 -c 2

Femto Bolt
roslaunch vibro_tactile_toolbox femto_bolt.launch

Femto Bolt cropped
rosrun vibro_tactile_toolbox image_cropped_republisher.py

Terminator
rosrun vibro_tactile_toolbox terminator_node.py