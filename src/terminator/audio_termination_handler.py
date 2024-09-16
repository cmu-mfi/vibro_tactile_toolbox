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

    def get_audio_segment(self, t_target, lagging_buffer=0.5, leading_buffer=0.5):
        # Convert buffer to a numpy array for processing
        # find t_target
        # find t_target - lagging_buffer
        # find t_target + leading_buffer
        start_time = t_target - rospy.Duration(lagging_buffer)

        current_buffer = list(self.buffer)

        timestamps = [stamp for (stamp, _) in current_buffer]

        closest_timestamp_index = np.argmin(np.abs(np.array(timestamps) - start_time.to_sec()))
        print(start_time.to_sec())
        print(current_buffer[closest_timestamp_index][0])

        audio_buffer = np.hstack(tuple([buffer for (stamp,buffer) in current_buffer[closest_timestamp_index:]]))
        
        # Process the audio data as needed
        audio_segment, rgb_images = segment_audio(audio_buffer, self.sample_rate, 0.0, lagging_buffer+leading_buffer)
        return audio_segment, rgb_images


class AudioTerminationHandler(BaseTerminationHandler):
    def __init__(self):
        super().__init__()
        # Input data buffer
        sample_rate = 44100
        buffer_duration_s = 10.0

        self.audio_buffer = AudioBuffer(sample_rate, buffer_duration_s)

        self.check_rate_ns = 10E6 # 10 ms default
        self.model = None


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
                self.check_rate_ns = pose_cfg['check_rate_ns']
            if 'model_path' in audio_cfg:
                self.model = torch.jit.load(cfg_json['model_path'])
                self.model.eval()
                self.model.to(device)
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

        audio_segment, rgb_images = self.audio_buffer.get_audio_segment(rospy.Time.now(), 0.2, 0.0)

        X = torch.zeros([1,3*num_channels,201,45])

        combined_image = np.zeros((num_channels*201,45,3), dtype=np.uint8)
        for channel in range(num_channels):
            opencv_image = cv2.cvtColor(np.array(rgb_images[channel]), cv2.COLOR_RGBA2RGB)
            pil_image = Image.fromarray(opencv_image)
            #pil_image.save(f'/mnt/hdd1/test_{channel}.png')
            X[:,channel*3:(channel+1)*3,:,:] = transforms.ToTensor()(pil_image)
            cv_image = cv2.cvtColor(np.array(rgb_images[channel]), cv2.COLOR_RGBA2BGR)
            combined_image[channel*201:(channel+1)*201,:,:] = cv_image

        X = X.to(device)
        pred = self.model(X)
        cpu_pred = pred.to('cpu').detach().numpy()
        print(cpu_pred.reshape(-1,))
        label = int(np.argmax(cpu_pred.reshape(-1,)))
        print(label)

        terminate = (label == 1)

        resp.id = req.id
        resp.spec = combined_img_msg

        if terminate:
            cause = 'Audio termination handler caused by: predicted terminate.'
        
        termination_signal = TerminationSignal()
        termination_signal.id = self.id
        termination_signal.terminate = terminate
        termination_signal.cause = cause
        return termination_signal