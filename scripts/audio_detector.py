#!/usr/bin/env python3

import os
import time
import signal
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
import torchaudio
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import matplotlib.pyplot as plt
import os
from PIL import Image

device = 'cuda' if torch.cuda.is_available() else 'cpu'
print('Using {} device'.format(device))

def segment_audio(audio_data, sample_rate, t_start, t_end):
    """
    Segment audio and save spectrogram images
    
    audio_data, sample_rate = torchaudio.load(filepath)
    """

    start_idx = int(t_start * sample_rate)
    end_idx = int(t_end * sample_rate)

    audio_ch0 = audio_data[0, :]
    audio_ch1 = audio_data[1, :]

    audio_segment = audio_ch0[start_idx:end_idx]
    transform = torchaudio.transforms.Spectrogram()
    spec_tensor = transform(torch.tensor(audio_segment))
    spec_np = spec_tensor.log2().numpy()
    spec_np = np.flipud(spec_np)

    # Begin from matplotlib.image.imsave
    sm = cm.ScalarMappable(cmap='viridis')
    sm.set_clim(None, None)
    rgba = sm.to_rgba(spec_np, bytes=True)
    pil_shape = (rgba.shape[1], rgba.shape[0])
    image_rgba = Image.frombuffer(
            "RGBA", pil_shape, rgba, "raw", "RGBA", 0, 1)
    # End from matplotlib.image.imsave
    #image_rgb = image_rgba.convert('rgb')

    return audio_segment, image_rgba

class CNNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Conv2d(3, 16, kernel_size=5)
        self.conv2 = nn.Conv2d(16, 32, kernel_size=5)
        self.conv2_drop = nn.Dropout2d()
        self.conv3 = nn.Conv2d(32, 16, kernel_size=5)
        self.flatten = nn.Flatten()
        self.fc1 = nn.Linear(8064, 512)
        self.fc2 = nn.Linear(512, 256)
        self.fc3 = nn.Linear(256, 3)


    def forward(self, x):
        x = F.relu(F.max_pool2d(self.conv1(x), 2))
        x = F.relu(F.max_pool2d(self.conv2_drop(self.conv2(x)), 2))
        x = F.relu(F.max_pool2d(self.conv3(x), 2))
        #x = x.view(x.size(0), -1)
        x = self.flatten(x)
        x = F.relu(self.fc1(x))
        x = F.dropout(x, training=self.training)
        x = F.relu(self.fc2(x))
        x = F.dropout(x, training=self.training)
        x = F.relu(self.fc3(x))
        return F.log_softmax(x,dim=1)


class AudioBuffer:
    def __init__(self, sample_rate, buffer_duration):
        self.sample_rate = sample_rate
        self.num_channels = 3
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
        print(closest_timestamp_index)
        print(start_time.to_sec())
        #deque_slice = collections.deque(itertools.islice(self.buffer, closest_timestamp_index, len(self.buffer)-1))
        print(current_buffer[closest_timestamp_index][0])

        audio_buffer = np.hstack(tuple([buffer for (stamp,buffer) in current_buffer[closest_timestamp_index:]]))
        
        # Process the audio data as needed
        audio_segment, image_rgb = segment_audio(audio_buffer, self.sample_rate, 0.0, lagging_buffer+leading_buffer)
        return audio_segment, image_rgb


class AudioDetector:

    def __init__(self, namespace):
        # Input data buffer
        sample_rate = 44100
        buffer_duration_s = 10.0

        self.audio_buffer = AudioBuffer(sample_rate, buffer_duration_s)
        self.audio_subscriber = rospy.Subscriber(f"/{namespace}/audio", AudioData, self.audio_buffer.callback)

        # Model config - torchaudio stuff

        self.bridge = CvBridge()
        self.model = CNNet()

        self.service = rospy.Service(f"/{namespace}/audio_detector", AudioOutcome, self.detect_audio)

        self.outcome_repub = rospy.Publisher(f"/{namespace}/outcome/audio_detector", AudioOutcomeRepub, queue_size=1)
        self.image_pub = rospy.Publisher(f"/{namespace}/outcome/audio_detector/spectrogram", sensor_msgs.msg.Image, queue_size=1)


    def detect_audio(self, req):
        print("Received Request")
        resp = AudioOutcomeResponse()
        self.model = torch.load(req.model_path)
        self.model.eval()
        self.model.to(device)

        audio_segment, image_rgb = self.audio_buffer.get_audio_segment(req.stamp)
        opencv_image = cv2.cvtColor(np.array(image_rgb), cv2.COLOR_RGBA2RGB)
        pil_image = Image.fromarray(opencv_image)
        cv_image = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
        
        pil_image_tensor = transforms.functional.to_tensor(pil_image)
        X = torch.reshape(pil_image_tensor, (1,3,201,221)).to(device)
        pred = self.model(X)
        cpu_pred = pred.to('cpu')
        label = int(np.argmax(cpu_pred.detach().numpy()))
        print(label)

        success = (label == 1)
        resp.result = json.dumps({'pred' : label,
                                  'success': success})
    
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

        resp.id = req.id
        resp.spec = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")

        # For rosbag record purposes
        outcome_repub_msg = AudioOutcomeRepub()
        outcome_repub_msg.id = resp.id
        outcome_repub_msg.spec = resp.spec
        outcome_repub_msg.result = resp.result
        self.outcome_repub.publish(outcome_repub_msg)

        return resp
                                                        
def main(args):
  namespace = rospy.get_namespace()
  namespace = namespace[1:-1]
  rospy.init_node(f"{namespace}_audio_detector", anonymous=True)
  node = AudioDetector(namespace)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)

