#!/usr/bin/env python3

import sys
import argparse
import rospy
import numpy as np
import json
from matplotlib import cm

import cv2
import sensor_msgs.msg

from cv_bridge import CvBridge
from vibro_tactile_toolbox.srv import AudioOutcome, AudioOutcomeResponse
from vibro_tactile_toolbox.msg import AudioOutcomeRepub

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
import soundfile as sf
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

    def get_audio_segment(self, id, channels, t_target, lagging_buffer=0.5, leading_buffer=0.5):
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
        audio_segment, rgb_images = segment_audio(channels, audio_buffer, self.sample_rate, 0.0, lagging_buffer+leading_buffer)
        return audio_segment, rgb_images


class AudioDetector:

    def __init__(self, namespace):
        # Input data buffer
        sample_rate = 44100
        buffer_duration_s = 10.0

        self.audio_buffer = AudioBuffer(sample_rate, buffer_duration_s)
        self.audio_subscriber = rospy.Subscriber(f"/{namespace}/audio", AudioData, self.audio_buffer.callback)

        # Model config - torchaudio stuff

        self.bridge = CvBridge()
        self.model = None
        self.service = rospy.Service(f"/{namespace}/audio_detector", AudioOutcome, self.detect_audio)

        self.outcome_repub = rospy.Publisher(f"/{namespace}/outcome/audio_detector", AudioOutcomeRepub, queue_size=1)
        self.image_pub = rospy.Publisher(f"/{namespace}/outcome/audio_detector/spectrogram", sensor_msgs.msg.Image, queue_size=1)


    def detect_audio(self, req):
        print("Received Request")
        resp = AudioOutcomeResponse()
        self.model = torch.jit.load(req.model_path)
        print(req.model_path)
        if 'outcome' in req.model_path:
            model_type = 'outcome'
        elif 'recovery' in req.model_path:
            model_type = 'recovery'
        self.model.eval()
        self.model.to(device)

        num_channels = len(req.channels)

        audio_segment, rgb_images = self.audio_buffer.get_audio_segment(req.id, req.channels, req.stamp, 0.7, 0.3)

        X = torch.zeros([1,num_channels,256,87])

        for channel in req.channels:
            X[:,channel,:,:] = torch.from_numpy(rgb_images[channel])

        X = X.to(device)
        pred = self.model(X)
        cpu_pred = pred.to('cpu').detach().numpy()

        if model_type == 'outcome':
            print(cpu_pred.reshape(-1,))
            label = int(np.argmax(cpu_pred.reshape(-1,)))
            print(label)

            success = (label == 1)
            resp.result = json.dumps({'result' : f"Audio Model predicted: {label}",
                                      'success': success})
        elif model_type == 'recovery':
            print(cpu_pred.reshape(-1,))
            action = cpu_pred.reshape(-1,).tolist()
            print(action)
            resp.result = json.dumps({'action' : action})
    
        #combined_img_msg = self.bridge.cv2_to_imgmsg(combined_image, "bgr8")
        #self.image_pub.publish(combined_img_msg)

        resp.id = req.id
        #resp.spec = combined_img_msg

        # For rosbag record purposes
        outcome_repub_msg = AudioOutcomeRepub()
        outcome_repub_msg.id = resp.id
        #outcome_repub_msg.spec = resp.spec
        outcome_repub_msg.result = resp.result
        self.outcome_repub.publish(outcome_repub_msg)

        return resp
                                                        
def main(args):
  namespace = rospy.get_namespace()
  namespace = namespace[1:-1]
  rospy.init_node("audio_detector_node", anonymous=True)
  node = AudioDetector(namespace)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)

