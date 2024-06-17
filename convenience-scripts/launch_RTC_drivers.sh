#!/bin/bash

# Check if a tmux window exists
window_exists() {
        tmux list-windows -t "$1" | grep -q "$2"
}

# Create new tmux window if it doesn't exist
create_window() {
        if ! window_exists "$1" "$2"; then
                tmux new-window -n "$2" -t "$1"
        fi
}

#read -sp "Enter password for [mfi@mfi-twin]: " mfi_twin_pass
#echo -e ""
#read -sp "Enter password for [mfi@yk-god]: " yk_god_pass
#echo -e ""

# Create a new tmux session called "RTC-drivers" if not already running
if ! tmux has-session -t RTC-drivers 2>/dev/null; then
        tmux new-session -d -s RTC-drivers
fi


# Start roscore in the first window
# We'll have roscore seperate from the drivers
#create_window "RTC-drivers" "roscore"
#tmux send-keys -t RTC-drivers:roscore 'roscore' C-m

# Start the fts drivers in a seperate window
create_window "RTC-drivers" "fts"
tmux send-keys -t RTC-drivers:fts 'ssh yk-god' C-m
tmux send-keys -t RTC-drivers:fts 'rosrun fts_serial fts_serial_node' C-m

# Start a dummy screen (if screen not connected) in a seperate window
create_window "RTC-drivers" "dummy-screen"
tmux send-keys -t RTC-drivers:dummy-screen "ssh yk-god" C-m
tmux send-keys -t RTC-drivers:dummy-screen 'start_dummy_screen' C-m
tmux send-keys -t RTC-drivers:dummy-screen '$mfi_twin_pass' C-m

# Start Realsense Wrist Camera in a seperate window
create_window "RTC-drivers" "realsense"
tmux send-keys -t RTC-drivers:realsense 'ssh -X yk-god' C-m
tmux send-keys -t RTC-drivers:realsense 'export DISPLAY=:0' C-m
tmux send-keys -t RTC-drivers:realsense 'roslaunch realsense2_camera rs_aligned_depth.launch' C-m

# Start Femto Bolt Camera in a seperate window
create_window "RTC-drivers" "femtobolt"
tmux send-keys -t RTC-drivers:femtobolt 'ssh -X yk-god' C-m
tmux send-keys -t RTC-drivers:femtobolt 'export DISPLAY=:0' C-m
tmux send-keys -t RTC-drivers:femtobolt 'roslaunch vibro_tactile_toolbox femto_bolt.launch' C-m

# Start Femto Bolt Cropper in a seperate window
create_window "RTC-drivers" "femtobolt-cropper"
tmux send-keys -t RTC-drivers:femtobolt-cropper 'ssh yk-god' C-m
tmux send-keys -t RTC-drivers:femtobolt-cropper 'rosrun vibro_tactile_toolbox image_cropped_republisher.py' C-m

# Start Audio in a seperate window
create_window "RTC-drivers" "audio"
tmux send-keys -t RTC-drivers:audio 'ssh yk-god' C-m
tmux send-keys -t RTC-drivers:audio 'rosrun sounddevice_ros sounddevice_ros_publisher_node.py -d 2 -c 2' C-m

# Start Robot in a seperate window
# create_window "RTC-drivers" "yk-creator-moveit"
# tmux send-keys -t RTC-drivers:yk-creator-moveit 'roslaunch testbed_utils lego_moveit_yk.launch namespace:=yk_creator' C-m

# Start Robot Pose pub in a seperate window
# create_window "RTC-drivers" "yk-creator-pose"
# tmux send-keys -t RTC-drivers:yk-creator-pose 'rosrun vibro_tactile_toolbox pose_stamped_publisher.py -n yk_creator -f 100' C-m

# attach the tmux session to check
# dtmux attach -t RTC-drivers
