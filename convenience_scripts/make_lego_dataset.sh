#!/bin/bash

# Define the arrays for volumes, bricks, and velocities
VOLS=(75)
BRICKS=("2x1" "2x2" "4x1" "4x2") 
VELS=(0.01 0.02)
TRAIN_VS_TEST=("vel_" "test_vel_")
ROBOT_NAME=""
TYPE="lego"
NUM_RESAMPLES=20
NUM_RESAMPLES_TERMINATOR=50

# Define the base paths
BASE_TARGET_DIR="${DATA_DIR}/lego/volume_"
BASE_TARGET_DST="${DATA_DIR}/lego_dataset/volume_"

# Iterate through each combination of volume, brick, and velocity
for VOL in "${VOLS[@]}"; do
  for BRICK in "${BRICKS[@]}"; do
    for VEL in "${VELS[@]}"; do
      for DATA_TYPE in "${TRAIN_VS_TEST[@]}"; do
        # Construct the directory paths
        TARGET_DIR="${BASE_TARGET_DIR}${VOL}/${BRICK}/${DATA_TYPE}${VEL}"
        TARGET_DST="${BASE_TARGET_DST}${VOL}/${BRICK}/${DATA_TYPE}${VEL}"
        
        # Call the python script with the constructed paths
        python ${PROJ_DIR}/scripts/parse_rosbag.py -w "$TARGET_DIR" -d "$TARGET_DIR" -n ${ROBOT_NAME}
        python ${PROJ_DIR}/model_training/create_outcome_training_dataset.py -s "$TARGET_DIR" -d "$TARGET_DST" -t ${TYPE} -n ${NUM_RESAMPLES}
        python ${PROJ_DIR}/model_training/create_terminator_training_dataset.py -s "$TARGET_DIR" -d "$TARGET_DST" -t ${TYPE} -n ${NUM_RESAMPLES_TERMINATOR}
      done
    done
  done
done
