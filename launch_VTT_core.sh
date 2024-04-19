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

# Create a new tmux session called "RTC-drivers" if not already running
if ! tmux has-session -t VTT-core 2>/dev/null; then
        tmux new-session -d -s VTT-core
fi

# Start the terminator
create_window "VTT-core" "terminator"
tmux send-keys -t VTT-core:terminator 'roslaunch vibro_tactile_toolbox terminators.launch' C-m

# Start the outcomer
create_window "VTT-core" "fts_detector"
tmux send-keys -t VTT-core:fts_detector 'roscd vibro_tactile_toolbox' C-m 
tmux send-keys -t VTT-core:fts_detector 'python3 scripts/fts_detector.py -t /fts' C-m 

create_window "VTT-core" "lego_detector"
tmux send-keys -t VTT-core:lego_detector 'roscd vibro_tactile_toolbox' C-m 
tmux send-keys -t VTT-core:lego_detector 'echo see README for launch instructions' C-m 

# WIP 
# Error after ./run ...
# "access control disabled, clients can connect from any host"
# "xhost: must be on local machien to enable or disable access control"

tmux send-keys -t VTT-core:lego_detector 'roscd vibro_tactile_toolbox' C-m 
tmux send-keys -t VTT-core:lego_detector 'cd docker' C-m 
tmux send-keys -t VTT-core:lego_detector './run  -i noetic_vibro_tactile_toolbox -c lego_detector -g' C-m 
tmux send-keys -t VTT-core:lego_detector 'source /ros1_ws/devel/setup.bash' C-m 
tmux send-keys -t VTT-core:lego_detector 'cd /home/repos/ros1_ws/src/kevin/vibro_tactile_toolbox/' C-m 
tmux send-keys -t VTT-core:lego_detector 'git pull' C-m
tmux send-keys -t VTT-core:lego_detector 'python3 scripts/lego_detector.py'  C-m 

# attach the tmux session to check
tmux attach -t VTT-core