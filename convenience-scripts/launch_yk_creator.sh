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
if ! tmux has-session -t yk-creator 2>/dev/null; then
        tmux new-session -d -s yk-creator
fi

# Start moveit
create_window "yk-creator" "moveit"
tmux send-keys -t yk-creator:moveit 'roslaunch testbed_utils lego_moveit_yk.launch namespace:=yk_creator' C-m

# Start the pose republisher
create_window "yk-creator" "pose_stamped_publisher"
tmux send-keys -t yk-creator:pose_stamped_publisher 'rosrun vibro_tactile_toolbox pose_stamped_publisher.py -n yk_creator -f 100' C-m 

# Attach the session to check
# tmux attach -t yk-creator