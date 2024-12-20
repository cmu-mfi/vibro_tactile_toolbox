#!/usr/bin/env python3
import argparse
import os
import shutil

import json
import numpy as np
from scipy import signal
from scipy.io import wavfile
import soundfile as sf

from PIL import Image
import librosa
import torch

import matplotlib
matplotlib.use('Agg')
import torchaudio

AUDIO_NAME = 'audio.wav'
AUDIO_NPY_NAME = 'audio.npy'
FTS_NAME = 'fts.npy'
SIDE_CAM_NAME = 'side_camera.mp4'
WRIST_CAM_NAME = 'wrist_camera.mp4'
TERMINATIONS_NAME = 'termination_signals.txt'
VISION_OUTCOMES_NAME = 'lego_outcomes.txt'
FTS_OUTCOMES_NAME = 'fts_outcomes.txt'
SKILL_NAME = 'skill_params.txt'


def string_to_bool(s: str) -> bool:
    if s.lower() == 'true':
        return True
    if s.lower() == 'false':
        return False
    else:
        return None

def is_valid_dataset(dataset_dir):
    """
    Determine if a directory is a valid 'parsed dataset' (matches expected output of scripts/parse_rosbag.py)
    """
    if not os.path.isdir(dataset_dir):
        return (False, [])
    print(f"Checking {dataset_dir}")
    expected_files = [AUDIO_NAME, FTS_NAME, SIDE_CAM_NAME, WRIST_CAM_NAME, TERMINATIONS_NAME, VISION_OUTCOMES_NAME, FTS_OUTCOMES_NAME]
    existing_files = os.listdir(dataset_dir)
    missing_files = []
    for f in expected_files:
        if f not in existing_files:
            print(f"Missing file: {f}")
            missing_files.append(f)
    return (True, missing_files)

def segment_audio(audio_data, sample_rate, t_start, t_end, audio_segment_length=0.2, resample_num=20, time_offset=0.2, sample_time_range=0.2):
    """
    Segment audio and save spectrogram images
    
    audio_data, sample_rate = torchaudio.load(filepath)
    """
    start_idx = int(t_start * sample_rate)
    end_idx = int(t_end * sample_rate)

    num_channels = audio_data.shape[0]
    
    audio_segment = audio_data[:,start_idx:end_idx]
    nominal_rgb_images = []
    terminate_rgb_images = []

    num_nominal = int(resample_num/2)
    num_terminate = int(resample_num/2)
    nominal_start_time = t_start
    nominal_end_time = t_end - time_offset - sample_time_range - audio_segment_length
    nominal_time_length = nominal_end_time - nominal_start_time
    terminate_start_time = t_end - time_offset - sample_time_range
    terminate_end_time = t_end - time_offset + sample_time_range - audio_segment_length
    terminate_time_length = terminate_end_time - terminate_start_time

    for i in range(num_nominal):
        random_start = np.random.random()*(nominal_time_length) + nominal_start_time
        start_idx = int((random_start) * sample_rate)
        end_idx = int((random_start + audio_segment_length) * sample_rate)

        num_channels = audio_data.shape[0]
            
        current_audio_segment = audio_data[:,start_idx:end_idx]
        
        for ch_num in range(num_channels):
            channel_audio_segment = current_audio_segment[ch_num,:]
            S = librosa.feature.melspectrogram(y=channel_audio_segment, sr=sample_rate, n_mels=256)
            S_dB = librosa.power_to_db(S, ref=np.max)

            nominal_rgb_images.append(S_dB)


    audio_segment = torch.from_numpy(audio_segment)

    for i in range(num_terminate):
        random_start = np.random.random()*(terminate_time_length) + terminate_start_time
        start_idx = int((random_start) * sample_rate)
        end_idx = int((random_start + audio_segment_length) * sample_rate)

        num_channels = audio_data.shape[0]
            
        current_audio_segment = audio_data[:,start_idx:end_idx]
        
        for ch_num in range(num_channels):
            channel_audio_segment = current_audio_segment[ch_num,:]

            S = librosa.feature.melspectrogram(y=channel_audio_segment, sr=sample_rate, n_mels=256)
            S_dB = librosa.power_to_db(S, ref=np.max)

            terminate_rgb_images.append(S_dB)

    return audio_segment, nominal_rgb_images, terminate_rgb_images

def segment_fts(fts_data, t_start, t_end):
    start_idx = np.searchsorted(fts_data[0, :], t_start)
    end_idx = np.searchsorted(fts_data[0, :], t_end)

    fts_segment = fts_data[start_idx:end_idx, :]
    return fts_segment

def segment_trial(dataset_dir, missing_files, desired_skill_names, audio_segment_length=0.2, num_resample=20, time_offset=0.2, sample_time_range=0.2):
    """
    Segment a trial into audio and fts data segments established by termination signals

    lagging_buffer: seconds behind terminal to segment
    leading_buffer: seconds ahead terminal to segment

    return: segments[]
    """

    # Load audio
    #audio_data, sample_rate = torchaudio.load(os.path.join(dataset_dir, AUDIO_NAME))
    audio_data = np.load(os.path.join(dataset_dir, AUDIO_NPY_NAME))
    sample_rate = 44100
    
    # Load FTS
    fts_data = np.load(os.path.join(dataset_dir, FTS_NAME))

    # Load termination_signals.txt
    terminals = []
    with open(os.path.join(dataset_dir, TERMINATIONS_NAME), 'r') as f:
        lines = f.readlines()
        for line in lines:
            if line.startswith('#'): continue
            entries = line.split(',')
            entries = [e.strip() for e in entries]
            # entries = [timestamp, skill_id, cause]
            timestamp, skill_id, termination_cause = float(entries[0]), int(entries[1]), entries[2]
            terminals.append((timestamp, skill_id, termination_cause))
    terminals.sort(key=lambda x: x[0])

    # Load skill_params.txt
    skill_names = {}
    with open(os.path.join(dataset_dir, SKILL_NAME), 'r') as f:
        lines = f.readlines()
        for line in lines:
            if line.startswith('#'): continue
            entries = line.split(',')
            entries = [e.strip() for e in entries]
            # entries = [timestamp, skill_id, skill_name, step_name]
            timestamp, skill_id, skill_name, step_name = float(entries[0]), float(entries[1]), entries[2], entries[3]
            skill_names[skill_id] = {'skill_name': skill_name, 'timestamp': timestamp}
    # Produce segments
    segments = []

    for (end_timestamp, skill_id, termination_cause) in terminals:
        skill_name = skill_names[skill_id]['skill_name']
        start_timestamp = skill_names[skill_id]['timestamp']
        if skill_name in desired_skill_names:

            audio_seg, nominal_rgb_images, terminate_rgb_images = segment_audio(audio_data, sample_rate, start_timestamp, end_timestamp, audio_segment_length, num_resample, time_offset, sample_time_range)
            fts_seg = segment_fts(fts_data, start_timestamp, end_timestamp)

            # Fill seg_data with the segmented data for saving later
            seg_data = {'audio':(sample_rate, audio_seg),
                        'nominal_spec': nominal_rgb_images,
                        'terminate_spec': terminate_rgb_images,
                        'fts': fts_seg}

            segments.append((skill_name, skill_id, seg_data))

    return segments

def make_file_names(dataset_name, skill_id, audio_dir, fts_dir, dataset_segment=None):
    '''
    returns: audio_pth, audio_spec_pth, fts_pth

    For now we assume all trials have 1 failed MoveDown, 1 successful MoveDown, 1 successful (LegoPick | LegoPlace)
    '''
    if dataset_segment:
        # audio_pth = os.path.join(audio_dir, f'seg_{segment_num}.wav')
        # audio_spec_pth = os.path.join(audio_dir, f'seg_{segment_num}.png')
        # fts_pth = os.path.join(fts_dir, f'seg_{segment_num}.npy')
        raise NotImplementedError
    audio_pth = os.path.join(audio_dir, f'{dataset_name}_{skill_id}.wav')
    audio_spec_pth = os.path.join(audio_dir, f'{dataset_name}_{skill_id}')
    fts_pth = os.path.join(fts_dir, f'{dataset_name}_{skill_id}.npy')
    return audio_pth, audio_spec_pth, fts_pth

def main(args):
    RECORDED_SKILLS = {'MoveDownToContact'} #, 'PickLego', 'PlaceLego'}
    # Create output folder 
    if not os.path.exists(args.save_dir):
        os.mkdir(args.save_dir) # if dir exists, delete it and remake

    # Instantiate dataset variables
    segment_num = 0
    audio_dir = 'audio'
    fts_dir = 'fts'

    for skill_name in (RECORDED_SKILLS | {'misc'}):
        if not os.path.exists(os.path.join(args.save_dir, skill_name)):
            os.mkdir(os.path.join(args.save_dir, skill_name))
        os.mkdir(os.path.join(args.save_dir, skill_name, audio_dir))
        os.mkdir(os.path.join(args.save_dir, skill_name, fts_dir))
        os.mkdir(os.path.join(args.save_dir, skill_name, 'nominal'))
        os.mkdir(os.path.join(args.save_dir, skill_name, 'terminate'))
        os.mkdir(os.path.join(args.save_dir, skill_name, 'nominal', audio_dir))
        os.mkdir(os.path.join(args.save_dir, skill_name, 'terminate', audio_dir))

    trials_pth = os.path.join(args.save_dir, 'terminator_trials.txt')
    labels_pth = os.path.join(args.save_dir, 'terminator_labels.txt')

    with open(trials_pth, 'x') as f:
        f.write("# Data collection trials used in this dataset. Names match corresponding rosbags\n")
        f.write("# dataset/rosbag name\n")

    with open(labels_pth, 'x') as f:
        f.write("# Labels for data segments.\n")
        f.write("# skill_name, audio_pth, audio_spec_pth, fts_pth\n")

    # Iterate through each trial and extract labelled segments of data
    for dataset_name in os.listdir(args.trial_src):
        dataset_dir = os.path.join(args.trial_src, dataset_name)
        # Check if dataset_dir is a valid 'parsed dataset' by seeing if the contents match
        # an expected file tree template
        dataset_info = is_valid_dataset(dataset_dir)
        if not dataset_info[0]:
            continue

        # Split trial into segments near terminal events
        segments = segment_trial(dataset_dir, dataset_info[1], desired_skill_names=RECORDED_SKILLS, audio_segment_length=args.audio_segment_length, num_resample=args.num_resample, time_offset=args.time_offset, sample_time_range=args.sample_time_range)

        # Save trial segments to dataset
        with open(trials_pth, 'a') as f:
            f.write(dataset_dir + '\n')

        for (skill_name, skill_id, seg_data) in segments:
            file_pths = make_file_names(dataset_name, skill_id, audio_dir, fts_dir)
            audio_pth = file_pths[0]
            audio_spec_pth = file_pths[1]
            fts_pth = file_pths[2]
    
            '''
            Produce this file structure:
            args.save_dir/
                labels.txt
                trials.txt
                MoveDown/
                    nominal/
                        audio/
                        fts/
                    terminate/
                        audio/
                        fts/
                misc/...
            '''


            if skill_name not in RECORDED_SKILLS:
                skill_name = 'misc'

            # Write labels.txt with label and corresponding segment files
            with open(labels_pth, 'a') as f:
                f.write(', '.join([skill_name,
                                   os.path.join(skill_name, audio_pth), 
                                   os.path.join(skill_name, audio_spec_pth), 
                                   os.path.join(skill_name, fts_pth)]) + '\n')

            # save audio segment
            if seg_data['audio'] is not None:
                audio_data = seg_data['audio'][1]
                sample_rate = seg_data['audio'][0]
                if len(audio_data.shape) < 2:
                    audio_data = audio_data[None, :]
                torchaudio.save(os.path.join(args.save_dir, skill_name, audio_pth), audio_data, sample_rate)
                #wavfile.write(os.path.join(args.save_dir, skill_name, audio_pth), seg_data['audio'][0], seg_data['audio'][1])
            # save audio segment spectrogram
            if seg_data['nominal_spec'] is not None:
                for (i,nominal_spec) in enumerate(seg_data['nominal_spec']):
                    np.save(os.path.join(args.save_dir, skill_name, 'nominal', audio_spec_pth + '_' + str(i) + '.npy'), nominal_spec)
            if seg_data['terminate_spec'] is not None:
                for (i,terminate_spec) in enumerate(seg_data['terminate_spec']):
                    np.save(os.path.join(args.save_dir, skill_name, 'terminate', audio_spec_pth + '_' + str(i) + '.npy'), terminate_spec)
            
            # save fts segment
            if seg_data['fts'] is not None:
                np.save(os.path.join(args.save_dir, skill_name, fts_pth), seg_data['fts'])

            segment_num += 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Creates a training dataset from multiple parsed VTT skill trials')
    parser.add_argument('--trial_src', '-s', type=str, required=True,
                        help='Path to folder containing parsed trial datasets')
    parser.add_argument('--save_dir', '-d', type=str, required=True,
                        help='Path to save trial dataset')
    parser.add_argument('--type', '-t', type=str, required=True,
                        help='Data Type')
    parser.add_argument('--time_offset', type=float, required=False, default=0.3,
                        help='Time to subtract from the terminate message. Default is 0.3s.')
    parser.add_argument('--audio_segment_length', type=float, required=False, default=0.5,
                        help='Specify the length of each audio segment. Default is 0.5s.')
    parser.add_argument('--sample_time_range', type=float, required=False, default=0.2,
                    help='Specify the range of sampling each sample. Default is 0.2s.')
    parser.add_argument('--num_resample', '-n', type=int, required=False, default=20,
                    help='Number of times to resample the audio segments.')
    args = parser.parse_args()

    main(args)