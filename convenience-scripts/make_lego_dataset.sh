#!/bin/bash

# Define the arrays for volumes, bricks, and velocities
VOLS=(75)
BRICKS=("2x1" "2x2" "2x4" "4x1" "4x2")
VELS=(0.01 0.02)

# Define the base paths
BASE_TARGET_DIR="/mnt/hdd1/vibrotactile_data/lego/volume_"
BASE_TARGET_DST="/mnt/hdd1/vibrotactile_data/lego_dataset/volume_"

# Iterate through each combination of volume, brick, and velocity
for VOL in "${VOLS[@]}"; do
  for BRICK in "${BRICKS[@]}"; do
    for VEL in "${VELS[@]}"; do
      # Construct the directory paths
      TARGET_DIR="${BASE_TARGET_DIR}${VOL}/${BRICK}/vel_${VEL}"
      TARGET_DST="${BASE_TARGET_DST}${VOL}/${BRICK}/vel_${VEL}"
      
      # Call the python script with the constructed paths
      python scripts/parse_rosbag.py -w "$TARGET_DIR" -d "$TARGET_DIR"
      python model_training/create_training_dataset.py -s "$TARGET_DIR" -d "$TARGET_DST"
    done
  done
done
