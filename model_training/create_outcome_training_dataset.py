#!/usr/bin/env python3
import argparse
import os
import shutil

import json
import numpy as np
from scipy import signal
from scipy.io import wavfile
import cv2
import soundfile as sf

import matplotlib.pyplot as plt
from matplotlib import cm
from PIL import Image
import librosa


import matplotlib
matplotlib.use('Agg')

import torchaudio
import torch

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
        return False
    print(f"Checking {dataset_dir}")
    expected_files = [AUDIO_NAME, FTS_NAME, SIDE_CAM_NAME, WRIST_CAM_NAME, TERMINATIONS_NAME, VISION_OUTCOMES_NAME, FTS_OUTCOMES_NAME]
    existing_files = os.listdir(dataset_dir)
    for f in expected_files:
        if f not in existing_files:
            print(f"Skipping {dataset_dir}. Missing file: {f}")
            return False
    return True

def segment_audio(audio_data, sample_rate, t_start, t_end, resample_num=0, time_offset=0.2):
    """
    Segment audio and save spectrogram images
    
    audio_data, sample_rate = torchaudio.load(filepath)
    """
    start_idx = int(t_start * sample_rate)
    end_idx = int(t_end * sample_rate)

    num_channels = audio_data.shape[0]

    audio_segment = audio_data[:,start_idx:end_idx]
    rgb_images = []
    for ch_num in range(num_channels):
        channel_audio_segment = audio_segment[ch_num,:]

        #fig=plt.figure()

        #ax = fig.gca()
        #D = librosa.amplitude_to_db(np.abs(librosa.stft(channel_audio_segment)), ref=np.max)
        S = librosa.feature.melspectrogram(y=channel_audio_segment, sr=sample_rate, n_mels=256)
        S_dB = librosa.power_to_db(S, ref=np.max)
        rgb_images.append(S_dB)

        # D = librosa.amplitude_to_db(np.abs(librosa.stft(channel_audio_segment)),ref=np.max)
        # print(D.shape)
        # rgb_images.append(D)
        # img = librosa.display.specshow(S_dB, sr=sample_rate, ax=ax)

        # fig.canvas.draw()  # Draw the canvas, cache the renderer

        # image_flat = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')  # (H * W * 3,)
        # # NOTE: reversed converts (W, H) from get_width_height to (H, W)
        # image = image_flat.reshape(*reversed(fig.canvas.get_width_height()), 3)  # (H, W, 3)
        # cropped_image = image[58:(58+370),80:(80+497),:]

        

        # fig.clear()
        # plt.close(fig)

    audio_segment = torch.from_numpy(audio_segment)

        # transform = torchaudio.transforms.Spectrogram()
        # spec_tensor = transform(channel_audio_segment)
        # spec_np = spec_tensor.log2().numpy()
        # spec_np = np.flipud(spec_np)

        # # Begin from matplotlib.image.imsave
        # sm = cm.ScalarMappable(cmap='viridis')
        # sm.set_clim(None, None)
        # rgba = sm.to_rgba(spec_np, bytes=True)
        # pil_shape = (rgba.shape[1], rgba.shape[0])
        # image_rgb = Image.frombuffer(
        #         "RGBA", pil_shape, rgba, "raw", "RGBA", 0, 1)
        # rgb_images.append(image_rgb)

        # End from matplotlib.image.imsave
    if resample_num > 0:
        for i in range(resample_num):
            random_offset = np.random.random()*(time_offset*2) - time_offset
            start_idx = int((t_start + random_offset) * sample_rate)
            end_idx = int((t_end + random_offset) * sample_rate)
            
            current_audio_segment = audio_data[:,start_idx:end_idx]
            
            for ch_num in range(num_channels):
                channel_audio_segment = current_audio_segment[ch_num,:]

                #fig=plt.figure()

                #ax = fig.gca()
                #D = librosa.amplitude_to_db(np.abs(librosa.stft(channel_audio_segment)), ref=np.max)
                S = librosa.feature.melspectrogram(y=channel_audio_segment, sr=sample_rate, n_mels=256)
                S_dB = librosa.power_to_db(S, ref=np.max)
                rgb_images.append(S_dB)

                # D = librosa.amplitude_to_db(np.abs(librosa.stft(channel_audio_segment)),ref=np.max)
                # rgb_images.append(D)
                # print(S_dB.shape)
                # img = librosa.display.specshow(S_dB, sr=sample_rate, ax=ax)

                # fig.canvas.draw()  # Draw the canvas, cache the renderer

                # image_flat = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')  # (H * W * 3,)
                # # NOTE: reversed converts (W, H) from get_width_height to (H, W)
                # image = image_flat.reshape(*reversed(fig.canvas.get_width_height()), 3)  # (H, W, 3)
                # cropped_image = image[58:(58+370),80:(80+497),:]

                

                # fig.clear()
                # plt.close(fig)

                # transform = torchaudio.transforms.Spectrogram()
                # spec_tensor = transform(channel_audio_segment)
                # spec_np = spec_tensor.log2().numpy()
                # spec_np = np.flipud(spec_np)

                # # Begin from matplotlib.image.imsave
                # sm = cm.ScalarMappable(cmap='viridis')
                # sm.set_clim(None, None)
                # rgba = sm.to_rgba(spec_np, bytes=True)
                # pil_shape = (rgba.shape[1], rgba.shape[0])
                # image_rgb = Image.frombuffer(
                #         "RGBA", pil_shape, rgba, "raw", "RGBA", 0, 1)
                # rgb_images.append(image_rgb)

                # End from matplotlib.image.imsave

    return audio_segment, rgb_images


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

def segment_trial(dataset_dir, lagging_buffer=0.5, leading_buffer=0.5, num_resample=20, time_offset=0.2):
    """
    Segment a trial into audio, fts, and vision data segments established by termination signals with outcome labels

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
            # entries = [timestamp, skill_id, cause]
            timestamp, skill_id, termination_cause = float(entries[0]), int(entries[1]), entries[2]
            terminals.append((timestamp, skill_id, termination_cause))
    terminals.sort(key=lambda x: x[0])

    # Load *_outcomes.txt
    outcomes = []
    with open(os.path.join(dataset_dir, VISION_OUTCOMES_NAME), 'r') as f:
        lines = f.readlines()
        for line in lines:
            if line.startswith('#'): continue
            entries = line.split(',')
            entries = [e.strip() for e in entries]
            # entries = [timestamp, result, success, outcome_ann]
            timestamp, result, success, outcome_ann = float(entries[0]), entries[1], entries[2], entries[3]
            if string_to_bool(success) is not None:
                outcomes.append((timestamp, success, outcome_ann))
    with open(os.path.join(dataset_dir, FTS_OUTCOMES_NAME), 'r') as f:
        lines = f.readlines()
        for line in lines:
            if line.startswith('#'): continue
            entries = line.split(',')
            entries = [e.strip() for e in entries]
            # entries = [timestamp, result, success]
            timestamp, result, success = float(entries[0]), entries[1], entries[2]     
            if string_to_bool(success) is not None:
                outcomes.append((timestamp, success, None))
    outcomes.sort(key=lambda x: x[0])

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
            skill_names[skill_id] = skill_name
    # Produce segments
    segments = []

    for i, outcome in enumerate(outcomes):
        t_outcome = outcome[0]
        t_terminals = [x[0] for x in terminals]
        i_detection_skill = np.searchsorted(t_terminals, t_outcome, 'right') - 1 # index of action to drive detection
        i_detected_skill = i_detection_skill - 1                                 # index of action outcome label corresponds to

        t_terminal = terminals[i_detected_skill][0]
        skill_id = terminals[i_detected_skill][1]
        skill_name = skill_names[skill_id] # get skill name from terminal id (matches skill id)

        # Segment data based on the terminal timestamp and lag/lead buffers
        t_start = t_terminal - lagging_buffer
        t_end = t_terminal + leading_buffer

        audio_seg, spec = segment_audio(audio_data, sample_rate, t_start, t_end, num_resample, time_offset)
        fts_seg = segment_fts(fts_data, t_start, t_end)
        side_cam_seg = segment_video(side_cam, t_terminal)
        wrist_cam_seg = segment_video(wrist_cam, t_terminal)

        if outcome[2]:
            outcome_ann = Image.open(os.path.join(dataset_dir, outcome[2]), 'r')
        else:
            outcome_ann = None

        # Fill seg_data with the segmented data for saving later
        seg_data = {'audio':(sample_rate, audio_seg),
                    'audio_spec': spec,
                    'fts': fts_seg,
                    'side_cam': side_cam_seg,
                    'wrist_cam': wrist_cam_seg,
                    'outcome_ann': outcome_ann}
        
        # Get the label for the data from the outcome detector
        label = outcome[1]

        segments.append((skill_name, skill_id, seg_data, label))

    return segments

def make_file_names(dataset_name, skill_id, audio_dir, fts_dir, vision_dir, dataset_segment=None):
    '''
    returns: audio_pth, audio_spec_pth, fts_pth, side_cam_pth, wrist_cam_pth, outcome_ann_pth

    For now we assume all trials have 1 failed MoveDown, 1 successful MoveDown, 1 successful (LegoPick | LegoPlace)
    '''
    if dataset_segment:
        # audio_pth = os.path.join(audio_dir, f'seg_{segment_num}.wav')
        # audio_spec_pth = os.path.join(audio_dir, f'seg_{segment_num}.png')
        # fts_pth = os.path.join(fts_dir, f'seg_{segment_num}.npy')
        # side_cam_pth = os.path.join(vision_dir, f'side_cam_{segment_num}.png')
        # wrist_cam_pth = os.path.join(vision_dir, f'wrist_cam_{segment_num}.png')
        # outcome_ann_pth = os.path.join(vision_dir, f'outcome_ann_{segment_num}.png')
        raise NotImplementedError
    audio_pth = os.path.join(audio_dir, f'{dataset_name}_{skill_id}.wav')
    audio_spec_pth = os.path.join(audio_dir, f'{dataset_name}_{skill_id}')
    fts_pth = os.path.join(fts_dir, f'{dataset_name}_{skill_id}.npy')
    side_cam_pth = os.path.join(vision_dir, f'{dataset_name}_{skill_id}.png')
    wrist_cam_pth = os.path.join(vision_dir, f'{dataset_name}_{skill_id}.png')
    outcome_ann_pth = os.path.join(vision_dir, f'{dataset_name}_{skill_id}.png')
    return audio_pth, audio_spec_pth, fts_pth, side_cam_pth, wrist_cam_pth, outcome_ann_pth

def main(args):
    RECORDED_SKILLS = {'MoveDown'} #, 'PickLego', 'PlaceLego'}
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

    for skill_name in (RECORDED_SKILLS | {'misc'}):
        os.mkdir(os.path.join(args.save_dir, skill_name))
        os.mkdir(os.path.join(args.save_dir, skill_name, 'success'))
        os.mkdir(os.path.join(args.save_dir, skill_name, 'fail'))
        os.mkdir(os.path.join(args.save_dir, skill_name, 'success', audio_dir))
        os.mkdir(os.path.join(args.save_dir, skill_name, 'success', fts_dir))
        os.mkdir(os.path.join(args.save_dir, skill_name, 'success', vision_dir))
        os.mkdir(os.path.join(args.save_dir, skill_name, 'fail', audio_dir))
        os.mkdir(os.path.join(args.save_dir, skill_name, 'fail', fts_dir))
        os.mkdir(os.path.join(args.save_dir, skill_name, 'fail', vision_dir))

    trials_pth = os.path.join(args.save_dir, 'trials.txt')
    labels_pth = os.path.join(args.save_dir, 'labels.txt')

    with open(trials_pth, 'x') as f:
        f.write("# Data collection trials used in this dataset. Names match corresponding rosbags\n")
        f.write("# dataset/rosbag name\n")

    with open(labels_pth, 'x') as f:
        f.write("# Labels for data segments.\n")
        f.write("# label, skill_name, audio_pth, audio_spec_pth, fts_pth, side_cam_pth, wrist_cam_pth, outcome_ann_pth\n")

    # Iterate through each trial and extract labelled segments of data
    for dataset_name in os.listdir(args.trial_src):
        dataset_dir = os.path.join(args.trial_src, dataset_name)
        # Check if dataset_dir is a valid 'parsed dataset' by seeing if the contents match
        # an expected file tree template
        if not is_valid_dataset(dataset_dir):
            continue

        # Split trial into segments near terminal events
        segments = segment_trial(dataset_dir, lagging_buffer=args.lagging_buffer, leading_buffer=args.leading_buffer, num_resample=args.num_resample)

        # Save trial segments to dataset
        with open(trials_pth, 'a') as f:
            f.write(dataset_dir + '\n')

        for (skill_name, skill_id, seg_data, label) in segments:
            file_pths = make_file_names(dataset_name, skill_id, audio_dir, fts_dir, vision_dir)
            audio_pth = file_pths[0]
            audio_spec_pth = file_pths[1]
            fts_pth = file_pths[2]
            side_cam_pth = file_pths[3]
            wrist_cam_pth = file_pths[4]
            outcome_ann_pth = file_pths[5]
    
            '''
            Produce this file structure:
            args.save_dir/
                labels.txt
                trials.txt
                engage/
                    success/
                        audio/
                        fts/
                        vision/
                    fail/
                        audio/
                        fts/
                        vision/
                pick/...
                place/...
                misc/...
            '''


            if skill_name not in RECORDED_SKILLS:
                skill_name = 'misc'
            if args.type == 'lego':
                label_pth = 'success' if (label.lower() == 'true') else 'fail'
            elif args.type == 'nist':
                label_pth = 'success' if (label.lower() == 'false') else 'fail'
            subdir = os.path.join(skill_name, label_pth)

            # Write labels.txt with label and corresponding segment files
            with open(labels_pth, 'a') as f:
                f.write(', '.join([label, skill_name,
                                   os.path.join(subdir, audio_pth), 
                                   os.path.join(subdir, audio_spec_pth), 
                                   os.path.join(subdir, fts_pth), 
                                   os.path.join(subdir, side_cam_pth), 
                                   os.path.join(subdir, wrist_cam_pth), 
                                   os.path.join(subdir, outcome_ann_pth)]) + '\n')

            # save audio segment
            if seg_data['audio'] is not None:
                audio_data = seg_data['audio'][1]
                sample_rate = seg_data['audio'][0]
                if len(audio_data.shape) < 2:
                    audio_data = audio_data[None, :]
                torchaudio.save(os.path.join(args.save_dir, subdir, audio_pth), audio_data, sample_rate)
                #wavfile.write(os.path.join(args.save_dir, subdir, audio_pth), seg_data['audio'][0], seg_data['audio'][1])
            # save audio segment spectrogram
            if seg_data['audio_spec'] is not None:
                for (i,audio_spec) in enumerate(seg_data['audio_spec']):
                    np.save(os.path.join(args.save_dir, subdir, audio_spec_pth + '_' + str(i) + '.npy'), audio_spec)
                    #opencv_image = cv2.cvtColor(np.array(audio_spec), cv2.COLOR_RGBA2BGR)
                    #opencv_image = cv2.cvtColor(np.array(audio_spec), cv2.COLOR_RGBA2RGB)
                    #pil_image = Image.fromarray(opencv_image)
                    #pil_image.save(os.path.join(args.save_dir, subdir, audio_spec_pth + '_' + str(i) + '.png'))
                    #cv2.imwrite(os.path.join(args.save_dir, subdir, audio_spec_pth + '_' + str(i) + '.png'), opencv_image)
                    #audio_spec.save()
            
            # save fts segment
            if seg_data['fts'] is not None:
                np.save(os.path.join(args.save_dir, subdir, fts_pth), seg_data['fts'])

            # save side_cam img
            if seg_data['side_cam'] is not None:
                cv2.imwrite(os.path.join(args.save_dir, subdir, side_cam_pth), seg_data['side_cam'])
            # save wrist_cam img
            if seg_data['wrist_cam'] is not None:
                cv2.imwrite(os.path.join(args.save_dir, subdir, wrist_cam_pth), seg_data['wrist_cam'])
            # save outcome_ann img
            if seg_data['outcome_ann'] is not None:
                seg_data['outcome_ann'].save(os.path.join(args.save_dir, subdir, outcome_ann_pth))

            segment_num += 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Creates a training dataset from multiple parsed VTT skill trials')
    parser.add_argument('--trial_src', '-s', type=str, required=True,
                        help='Path to folder containing parsed trial datasets')
    parser.add_argument('--save_dir', '-d', type=str, required=True,
                        help='Path to save trial dataset')
    parser.add_argument('--type', '-t', type=str, required=True,
                        help='Data Type')
    parser.add_argument('--leading_buffer', type=float, required=False, default=0.3,
                        help='Specify the seconds ahead of terminals to segment data. Default is 0.3s')
    parser.add_argument('--lagging_buffer', type=float, required=False, default=0.7,
                    help='Specify the seconds behind terminals to segment data. Default is 0.7s')
    parser.add_argument('--num_resample', '-n', type=int, required=False, default=20,
                    help='Number of times to resample the audio segments.')
    args = parser.parse_args()

    main(args)