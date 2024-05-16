#!/usr/bin/env python3
import argparse
import os
import shutil

import numpy as np
from scipy import signal
from scipy.io import wavfile
import cv2
import soundfile as sf

import matplotlib.pyplot as plt
from PIL import Image

AUDIO_NAME = 'audio.wav'
FTS_NAME = 'fts.npy'
SIDE_CAM_NAME = 'side_camera.mp4'
WRIST_CAM_NAME = 'wrist_camera.mp4'
TERMINATIONS_NAME = 'termination_signals.txt'
OUTCOMES_NAME = 'lego_outcomes.txt'

def is_valid_dataset(dataset_dir):
    """
    Determine if a directory is a valid 'parsed dataset' (matches expected output of scripts/parse_rosbag.py)
    """
    if not os.path.isdir(dataset_dir):
        return False
    print(f"Checking {dataset_dir}")
    expected_files = [AUDIO_NAME, FTS_NAME, SIDE_CAM_NAME, WRIST_CAM_NAME, TERMINATIONS_NAME, OUTCOMES_NAME]
    existing_files = os.listdir(dataset_dir)
    for f in expected_files:
        if f not in existing_files:
            print(f"Skipping {dataset_dir}. Missing file: {f}")
            return False
    return True

def segment_audio(audio_data, sample_rate, t_start, t_end):
    start_idx = int(t_start * sample_rate)
    end_idx = int(t_end * sample_rate)

    audio_segment = audio_data[start_idx:end_idx, 0]

    freq, time, audio_spec = signal.spectrogram(audio_segment, sample_rate)

    # Plot spectrogram
    fig, ax = plt.subplots()
    im = ax.pcolormesh(time, freq, 10 * np.log10(audio_spec))  # Convert to dB scale
    ax.set_ylabel('Frequency [Hz]')
    ax.set_xlabel('Time [s]')
    fig.colorbar(im, label='Intensity [dB]', ax=ax)
    ax.set_title('Spectrogram')
    fig.tight_layout()

    fig.canvas.draw()
    image_arr = np.array(fig.canvas.renderer.buffer_rgba())

    plt.close()

    image_rgb = Image.fromarray(image_arr)
    image_rgb = image_rgb.convert('RGB')
    return audio_segment, image_rgb


def segment_video(video_data: cv2.VideoCapture, t_terminal):
    """
    We won't be using 'segments of video' for training, this is just to get a ~picture~ of what's going on at the time inference
    would be called using the segmented audio data
    """
    video_data.set(cv2.CAP_PROP_POS_MSEC, 1000 * t_terminal)
    ret, frame = video_data.read()
    return frame

def segment_fts(fts_data, t_start, t_end):
    start_idx = np.searchsorted(fts_data[0, :], t_start)
    end_idx = np.searchsorted(fts_data[0, :], t_end)

    fts_segment = fts_data[start_idx:end_idx, :]
    return fts_segment

def segment_trial(dataset_dir, lagging_buffer=0.5, leading_buffer=0.5):
    """
    Segment a trial into audio, fts, and vision data segments established by termination signals with outcome labels

    lagging_buffer: seconds behind terminal to segment
    leading_buffer: seconds ahead terminal to segment

    return: segments[]
    """

    # Load audio
    sample_rate, audio_data = wavfile.read(os.path.join(dataset_dir, AUDIO_NAME))
    
    # Load FTS
    fts_data = np.load(os.path.join(dataset_dir, FTS_NAME))
    
    # Load vision
    side_cam = cv2.VideoCapture(os.path.join(dataset_dir, SIDE_CAM_NAME))
    wrist_cam = cv2.VideoCapture(os.path.join(dataset_dir, WRIST_CAM_NAME))

    # Load termination_signals.txt
    terminals = []
    with open(os.path.join(dataset_dir, TERMINATIONS_NAME), 'r') as f:
        lines = f.readlines()
        for line in lines:
            if line.startswith('#'): continue
            entries = line.split(',')
            entries = [e.strip() for e in entries]
            timestamp, termination_cause = entries[0], entries[1]
            terminals.append((timestamp, termination_cause))
    terminals.sort(key=lambda x: x[0])

    # Load outcomes.txt
    outcomes = []
    with open(os.path.join(dataset_dir, OUTCOMES_NAME), 'r') as f:
        lines = f.readlines()
        for line in lines:
            if line.startswith('#'): continue
            entries = line.split(',')
            entries = [e.strip() for e in entries]
            timestamp, outcome, outcome_ann = entries[0], entries[1], entries[2]
            outcomes.append((timestamp, outcome, outcome_ann))
    outcomes.sort(key=lambda x: x[0])

    # Produce segments
    segments = []
    for i, outcome in enumerate(outcomes):
        OUTCOME_TERMINAL_MAP = {0: 1, 1:3, 2:6}
        # # Get timestamp for a labelled outcome
        # t_outcome = float(outcome[0])

        # # Find nearest terminal prior to outcome
        # t_terminal = terminals[-1][0]
        # for terminal in terminals[::-1]:
        #     t_terminal = float(terminal[0])
        #     if t_terminal < t_outcome:
        #         break

        # TODO: Make outcome ids to match corresponding terminal/skill step ids
        t_terminal = float(terminals[OUTCOME_TERMINAL_MAP[i]][0])

        # Segment data based on the terminal timestamp and lag/lead buffers
        t_start = t_terminal - lagging_buffer
        t_end = t_terminal + leading_buffer

        audio_seg, spec = segment_audio(audio_data, sample_rate, t_start, t_end)
        fts_seg = segment_fts(fts_data, t_start, t_end)
        side_cam_seg = segment_video(side_cam, t_terminal)
        wrist_cam_seg = segment_video(wrist_cam, t_terminal)

        outcome_ann = Image.open(os.path.join(dataset_dir, outcome[2]), 'r')

        # Fill seg_data with the segmented data for saving later
        seg_data = {'audio':(sample_rate, audio_seg),
                    'audio_spec': spec,
                    'fts': fts_seg,
                    'side_cam': side_cam_seg,
                    'wrist_cam': wrist_cam_seg,
                    'outcome_ann': outcome_ann}
        
        # Get the label for the data from the outcome detector
        label = outcome[1]

        segments.append((seg_data, label))

    return segments



def main(args):
    # Create output folder 
    if os.path.exists(args.save_dir):
        if 'y' not in input(f'{args.save_dir} already exists, delete? [y/n] ').lower():
            return
        shutil.rmtree(args.save_dir)
    os.mkdir(args.save_dir) # if dir exists, delete it and remake

    # Instantiate dataset variables
    segment_num = 0
    audio_dir = 'audio'
    fts_dir = 'fts'
    vision_dir = 'vision'

    os.mkdir(os.path.join(args.save_dir, audio_dir))
    os.mkdir(os.path.join(args.save_dir, fts_dir))
    os.mkdir(os.path.join(args.save_dir, vision_dir))

    trials_pth = os.path.join(args.save_dir, 'trials.txt')
    labels_pth = os.path.join(args.save_dir, 'labels.txt')

    with open(trials_pth, 'x') as f:
        f.write("# Data collection trials used in this dataset. Names match corresponding rosbags\n")
        f.write("# dataset/rosbag name\n")

    with open(labels_pth, 'x') as f:
        f.write("# Labels for data segments.\n")
        f.write("# label, audio_pth, audio_spec_pth, fts_pth, side_cam_pth, wrist_cam_pth, outcome_ann_pth\n")

    # Iterate through each trial and extract labelled segments of data
    for dataset_name in os.listdir(args.trial_src):
        dataset_dir = os.path.join(args.trial_src, dataset_name)
        # Check if dataset_dir is a valid 'parsed dataset' by seeing if the contents match
        # an expected file tree template
        if not is_valid_dataset(dataset_dir):
            continue

        # Split trial into segments near terminal events
        segments = segment_trial(dataset_dir)

        # Save trial segments to dataset
        with open(trials_pth, 'a') as f:
            f.write(dataset_dir + '\n')

        for segment in segments:
            audio_pth = os.path.join(audio_dir, f'seg_{segment_num}.wav')
            audio_spec_pth = os.path.join(audio_dir, f'seg_{segment_num}.png')
            fts_pth = os.path.join(fts_dir, f'seg_{segment_num}.npy')
            side_cam_pth = os.path.join(vision_dir, f'side_cam_{segment_num}.png')
            wrist_cam_pth = os.path.join(vision_dir, f'wrist_cam_{segment_num}.png')
            outcome_ann_pth = os.path.join(vision_dir, f'outcome_ann_{segment_num}.png')

            seg_data = segment[0]
            label = segment[1]

            # Write labels.txt with label and corresponding segment files
            with open(labels_pth, 'a') as f:
                f.write(', '.join([label, audio_pth, audio_spec_pth, fts_pth, side_cam_pth, wrist_cam_pth, outcome_ann_pth]) + '\n')

            # save audio segment
            if seg_data['audio'] is not None:
                wavfile.write(os.path.join(args.save_dir, audio_pth), seg_data['audio'][0], seg_data['audio'][1])
            # save audio segment spectrogram
            if seg_data['audio_spec'] is not None:
                seg_data['audio_spec'].save(os.path.join(args.save_dir, audio_spec_pth), format='JPEG')
            
            # save fts segment
            if seg_data['fts'] is not None:
                np.save(os.path.join(args.save_dir, fts_pth), seg_data['fts'])

            # save side_cam img
            if seg_data['side_cam'] is not None:
                cv2.imwrite(os.path.join(args.save_dir, side_cam_pth), seg_data['side_cam'])
            # save wrist_cam img
            if seg_data['wrist_cam'] is not None:
                cv2.imwrite(os.path.join(args.save_dir, wrist_cam_pth), seg_data['wrist_cam'])
            # save outcome_ann img
            if seg_data['outcome_ann'] is not None:
                seg_data['outcome_ann'].save(os.path.join(args.save_dir, outcome_ann_pth), format='JPEG')

            segment_num += 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Creates a training dataset from multiple parsed VTT skill trials')
    parser.add_argument('--trial_src', '-s', type=str, required=True,
                        help='Path to folder containing parsed trial datasets')
    parser.add_argument('--save_dir', '-d', type=str, required=True,
                        help='Path to save trial dataset')
    args = parser.parse_args()

    main(args)