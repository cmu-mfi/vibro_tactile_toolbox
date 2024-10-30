#!/bin/bash

# Define the arrays for types
TYPES=("lego")
# "dsub" "usb" "lego" "waterproof"
CHANNELS=("0" "1" "2" "3" "0,1" "0,2" "0,3" "1,2" "1,3" "2,3" "0,1,2" "0,1,3" "1,2,3")

# Iterate through each combination of volume, brick, and velocity
for TYPE in "${TYPES[@]}"; do
  for CHANNEL in "${CHANNELS[@]}"; do
    python model_training/train_audio_outcome_model.py -t "${TYPE}" -c "${CHANNEL}"
  done
done