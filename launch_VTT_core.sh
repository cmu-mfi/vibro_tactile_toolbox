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

# attach the tmux session to check
tmux attach -t VTT-core