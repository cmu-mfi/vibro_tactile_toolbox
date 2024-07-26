# Model Training
This folder contains all the necessary functions and instructions for training machine learning models with the Vibro-Tactile Toolbox.

## Data Collection
Refer to the README.md in the root of the repository for running VTT skills for data collection.

To use our provided task script run:
```bash
roslaunch vibro_tactile_toolbox collect_tactile_data.launch
```
Which will save a rosbag of raw data to results/{bag_name}.bag

## Data Parsing
Raw rosbag trial data is then parsed into more portable data types for each trial.
Generally ROS messages are converted to *.mp4 for video, *.wav for audio, *.npy for
numerical data such as force torque and robot states, and *.txt files for termination and outcome messages.

Rosbags saved from collecting trial data can be parsed by running
```bash
python scripts/parse_rosbag.py -b $ROSBAG_PATH -d $SAVE_DST
```

This will create a parsed dataset with the following structure:
```bash
parsed_dataset/
├── termination_signals.txt
├── wrist_camera.mp4
├── side_camera.mp4
├── fts.npy
├── joint_states.npy
├── fts_outcomes.txt
├── vision_outcomes.txt
└── vision_outcomes/
│ ├── {outcome_ann0}.png
│ ├── ...
│ └── {outcome_annN}.png
```

## Data Visualization
Parsed datasets can be visualized/replayed using a QT GUI window. The window will display the collected data for a trial
along with recorded termination signals and outcome detections. The trial can be replayed by using the play button or dragging the time slider manually.
The playback is automatically paused when outcome detections are dispalyed. 

Display a parsed dataset by running
```bash
python scripts/display_dataset.py -s $PARSED_DATASET_PATH
```

## Creating Training Datasets
To use raw trial data to train neural-net classifiers, parsed datasets from multiple trials must be segmented, labelled, and collated.
A training dataset for vibro-tactile skills will contain a trials.txt file that records the parsed datasets used to produce the training set.
The training dataset will also contain unprocessed segments of data for each terminal or outcome 'event' used for training and a labels.txt file to match segments with outcome labels.
This is done so data can easily be loaded with torch.Dataset tooling but feature extraction/pre-processing is left to the engineer.

To produce a training dataset, run
```bash
python model_training/create_training_dataset.py --trial_src --dst
```
Where trial_src is a folder containing the target parsed datasets for collation. 

This will create a training dataset with the following structure:
```bash
training_dataset/
├── trials.txt
├── labels.txt
├── audio/
│ ├── {seg_0}.wav
│ ├── ...
│ └── {seg_N}.wav
├── fts/
│ ├── {seg_0}.npy
│ ├── ...
│ └── {seg_N}.npy
└── vision/
│ ├── {side_0}.png
│ ├── {wrist_0}.png
│ ├── {outcome_ann_0}.png
│ ├── ...
│ └── {outcome_ann_N}.png
```

## Loading Training Datasets with Torch
TODO: Adapt Kevin's script from [RobotAlert](https://github.com/RobotAlert/ModelTraining)

## Training
TODO: Get a general training procedure (not urgent)

### Audio Models
TODO: Adapt Kevin's script from [RobotAlert](https://github.com/RobotAlert/ModelTraining)

### Vision Models
TODO: Add docs for fine tuning detectron models (not urgent [detectron tutorial](https://detectron2.readthedocs.io/en/latest/tutorials/getting_started.html))

### FTS Models
Placeholder

### Multi-Modal Models
Placeholder
