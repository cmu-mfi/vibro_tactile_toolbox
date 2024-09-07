#!/bin/bash

# Define the arrays for volumes, connectors, and velocities
VOLS=(75)
CONNECTORS=("dsub")
VELS=(0.01 0.02)

# Define the base paths
BASE_TARGET_DIR="/mnt/hdd1/vibrotactile_data/nist/volume_"
BASE_TARGET_DST="/mnt/hdd1/vibrotactile_data/nist_dataset/volume_"

# Iterate through each combination of volume, brick, and velocity
for VOL in "${VOLS[@]}"; do
  for CONNECTOR in "${CONNECTORS[@]}"; do
    for VEL in "${VELS[@]}"; do
      # Construct the directory paths
      TARGET_DIR="${BASE_TARGET_DIR}${VOL}/${CONNECTOR}/vel_${VEL}"
      TARGET_DST="${BASE_TARGET_DST}${VOL}/${CONNECTOR}/vel_${VEL}"
      
      # Call the python script with the constructed paths
      python scripts/parse_rosbag.py -w "$TARGET_DIR" -d "$TARGET_DIR" -n "yk_builder"
      python model_training/create_training_dataset.py -s "$TARGET_DIR" -d "$TARGET_DST" 
    done
  done
done
