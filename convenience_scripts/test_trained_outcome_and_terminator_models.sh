#!/bin/bash

# Define the arrays for types
TYPES=("waterproof")
# "lego"
CHANNELS=("0,1,2,3")
# "0" "1" "2" "3" "0,1" "0,2" "0,3" "1,2" "1,3" "2,3" "0,1,2" "0,1,3" "0,2,3" "1,2,3"

# Iterate through each combination of volume, brick, and velocity
for TYPE in "${TYPES[@]}"; do
  for CHANNEL in "${CHANNELS[@]}"; do
    python ${PROJ_DIR}/model_training/test_trained_audio_outcome_model.py -t "${TYPE}" -c "${CHANNEL}" -d "${DATA_DIR}" -p "${PROJ_DIR}"
    python ${PROJ_DIR}/model_training/test_trained_audio_terminator_model.py -t "${TYPE}" -c "${CHANNEL}" -d "${DATA_DIR}" -p "${PROJ_DIR}"
  done
done
