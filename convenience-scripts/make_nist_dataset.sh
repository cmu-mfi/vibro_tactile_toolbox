#!/bin/bash

# Define the arrays for volumes, connectors, and velocities
VOLS=(75)
CONNECTORS=("dsub")
#"dsub" "waterproof" 
#"usb" "ethernet"
VELS=(0.01)
#0.01 0.02
TRAIN_VS_TEST=("test_vel_")
#"vel_" "test_vel_"
ROBOT_NAME="yk_builder"
TYPE="nist"
NUM_RESAMPLES=20
NUM_RESAMPLES_TERMINATOR=50

# Define the base paths
BASE_TARGET_DIR="/mnt/hdd1/vibrotactile_data/nist/volume_"
BASE_TARGET_DST="/home/mfi/Documents/vibrotactile_data/nist_dataset/volume_"

# Iterate through each combination of volume, brick, and velocity
for VOL in "${VOLS[@]}"; do
  for CONNECTOR in "${CONNECTORS[@]}"; do
    for VEL in "${VELS[@]}"; do
      for DATA_TYPE in "${TRAIN_VS_TEST[@]}"; do
        # Construct the directory paths
        TARGET_DIR="${BASE_TARGET_DIR}${VOL}/${CONNECTOR}/${DATA_TYPE}${VEL}"
        TARGET_DST="${BASE_TARGET_DST}${VOL}/${CONNECTOR}/${DATA_TYPE}${VEL}"
        
        # Call the python script with the constructed paths
        python scripts/parse_rosbag.py -w "$TARGET_DIR" -d "$TARGET_DIR" -n ${ROBOT_NAME}
        python model_training/create_outcome_training_dataset.py -s "$TARGET_DIR" -d "$TARGET_DST" -t ${TYPE} -n ${NUM_RESAMPLES}
        python model_training/create_terminator_training_dataset.py -s "$TARGET_DIR" -d "$TARGET_DST" -t ${TYPE} -n ${NUM_RESAMPLES_TERMINATOR}
      done
    done
  done
done
