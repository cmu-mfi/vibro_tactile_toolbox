#!/bin/bash

# Run me from /path/to/vibro_tactile_toolbox

# kill all old drivers
tmux kill-session -t RTC-drivers
tmux kill-session -t yk-creator
tmux kill-session -t VTT-core

bash convenience-scripts/launch_RTC_drivers.sh
bash convenience-scripts/launch_yk_creator.sh
bash convenience-scripts/launch_VTT_core.sh