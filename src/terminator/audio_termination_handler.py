#!/usr/bin/env python3

import rospy
import numpy as np
import json
from matplotlib import cm

import cv2
import sensor_msgs.msg

from sounddevice_ros.msg import AudioData

import collections
import itertools
from collections import deque

from torchvision import transforms
import torch
import torch.nn as nn
import torch.nn.functional as F
import torchaudio
import numpy as np
import matplotlib.pyplot as plt
import os
from PIL import Image
import cv2
from vibro_tactile_toolbox.msg import TerminationSignal, TerminationConfig
from terminator.base_termination_handler import BaseTerminationHandler

import terminator.utils as t_utils

import librosa

device = 'cuda' if torch.cuda.is_available() else 'cpu'
print('Using {} device'.format(device))


def segment_audio(channels, audio_data, sample_rate, t_start, t_end):
    """
    Segment audio and save spectrogram images
    
    audio_data, sample_rate = torchaudio.load(filepath)
    """
    start_idx = int(t_start * sample_rate)
    end_idx = int(t_end * sample_rate)

    num_channels = audio_data.shape[0]
    
    audio_segment = audio_data[:,start_idx:end_idx]
    rgb_images = []
    for ch_num in channels:
        channel_audio_segment = audio_segment[ch_num,:]

        S = librosa.feature.melspectrogram(y=channel_audio_segment, sr=sample_rate, n_mels=256)
        S_dB = librosa.power_to_db(S, ref=np.max)
        rgb_images.append(S_dB)

        # End from matplotlib.image.imsave

    return audio_segment, rgb_images

class AudioBuffer:
    def __init__(self, sample_rate, buffer_duration):
        self.sample_rate = sample_rate
        self.num_channels = 4
        self.buffer_duration = buffer_duration
        self.max_samples = int(200 * buffer_duration)
        self.buffer = deque(maxlen=self.max_samples)

    def callback(self, msg):
        # Assuming msg.data is a list of audio samples
        audio_samples = (msg.header.stamp.to_sec(), np.asarray(msg.data).reshape((-1,self.num_channels)).transpose())
        self.buffer.append(audio_samples)

    def get_last_audio_segment(self, channels, segment_length=0.5):
        # Convert buffer to a numpy array for processing
        # find t_target
        # find t_target - lagging_buffer
        # find t_target + leading_buffer

        audio_buffer = np.hstack(tuple([buffer for (stamp,buffer) in list(self.buffer)[-105:]]))
        
        # Process the audio data as needed
        audio_segment, rgb_images = segment_audio(channels, audio_buffer, self.sample_rate, 0.0, segment_length)
        return audio_segment, rgb_images


class AudioTerminationHandler(BaseTerminationHandler):
    def __init__(self):
        super().__init__()
        # Input data buffer
        sample_rate = 44100
        buffer_duration_s = 20.0

        self.audio_buffer = AudioBuffer(sample_rate, buffer_duration_s)

        self.input_data_class = AudioData
        self.check_rate_ns = 10E6 # 10 ms default
        self.model = None
        self.channels = [0,1,2,3]
        self.num_channels = len(self.channels)

    def update_config(self, cfg: TerminationConfig):
        """
        Update the termination handler config:
        - id
        - check_rate_ns
        - model_path
        """
        cfg_jsons = cfg.cfg_json
        cfg_json = json.loads(cfg_jsons)
        if 'audio' in cfg_json:
            self.live = True
            self.id = cfg_json['id']
            audio_cfg = cfg_json['audio']
            if 'check_rate_ns' in audio_cfg:
                self.check_rate_ns = audio_cfg['check_rate_ns']
            if 'model_path' in audio_cfg:
                self.model = torch.jit.load(audio_cfg['model_path'])
                self.model.eval()
                self.model.to(device)
            if 'channels' in audio_cfg:
                self.channels = audio_cfg['channels']
                self.num_channels = len(self.channels)
        else:
            self.live = False
    
    def update_input_data(self, input_signal: AudioData):
        """
        Extract the Pose from the most recent PoseStamped reading
        """
        self.audio_buffer.callback(input_signal)
    
    def get_termination_signal(self) -> TerminationSignal:
        """
        Create the termination signal and add causes based on the current pose
        """
        terminate = False
        cause = ''

        audio_segment, rgb_images = self.audio_buffer.get_last_audio_segment(self.channels, 0.5)

        X = torch.zeros([1,self.num_channels,256,44])
        try:
            for channel in self.channels:
                X[:,channel,:,:] = torch.from_numpy(rgb_images[channel])

            X = X.to(device)
            pred = self.model(X)
            cpu_pred = pred.to('cpu').detach().numpy()
            label = int(np.argmax(cpu_pred.reshape(-1,)))

            terminate = (label == 1)

            if terminate:
                cause = 'Audio termination handler caused by: predicted terminate.'
            
            termination_signal = TerminationSignal()
            termination_signal.id = self.id
            termination_signal.terminate = terminate
            termination_signal.cause = cause
            return termination_signal
        except:
            pass