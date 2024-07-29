#!/usr/bin/env python3

import os
import time
import signal
import sys
import argparse
import rospy
import numpy as np
import json

from vibro_tactile_toolbox.srv import AudioOutcome, AudioOutcomeResponse
from vibro_tactile_toolbox.msg import AudioOutcomeRepub

from sounddevice_ros.msg import AudioData

from collections import deque

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
    spec_tensor = transform(audio_segment)
    spec_np = spec_tensor.log2().numpy()
    spec_np = np.flipud(spec_np)

    # Begin from matplotlib.image.imsave
    sm = cm.ScalarMappable(cmap='viridis')
    sm.set_clim(None, None)
    rgba = sm.to_rgba(spec_np, bytes=True)
    pil_shape = (rgba.shape[1], rgba.shape[0])
    image_rgb = Image.frombuffer(
            "RGBA", pil_shape, rgba, "raw", "RGBA", 0, 1)
    # End from matplotlib.image.imsave

    return audio_segment, image_rgb


class AudioBuffer:
    def __init__(self, sample_rate, buffer_duration):
        self.sample_rate = sample_rate
        self.buffer_duration = buffer_duration
        self.max_samples = int(200 * buffer_duration)
        self.buffer = deque(maxlen=self.max_samples)

    def callback(self, msg):
        # Assuming msg.data is a list of audio samples
        audio_samples = (msg.header.stamp, msg.data)
        self.buffer.append(audio_samples)

    def get_audio_segment(self, t_target, lagging_buffer=0.5, leading_buffer=0.5):
        # Convert buffer to a numpy array for processing
        # find t_target
        # find t_target - lagging_buffer
        # find t_target + leading_buffer
        audio_data = np.array(self.buffer)
        # Process the audio data as needed



class AudioDetector:

    def __init__(self):
        # Input data buffer
        sample_rate = 44100
        buffer_duration_s = 5.0

        self.audio_buffer = AudioBuffer(sample_rate, buffer_duration_s)
        self.audio_subscriber = rospy.Subscriber('/audio', AudioData, self.audio_buffer.callback)

        # Model config - torchaudio stuff
        self.cfg = None
        self.predictor = None

        self.service = rospy.Service('audio_detector', AudioOutcome, self.detect_fts)

        self.outcome_repub = rospy.Publisher('/outcome/audio_detector', AudioOutcomeRepub, queue_size=1)


    def detect_audio(self, req):
        print("Received Request")
        resp = AudioOutcomeResponse()



        success = (len(result) > 0)
        resp.result = "nothing"

        # For rosbag record purposes
        outcome_repub_msg = AudioOutcomeRepub()
        outcome_repub_msg.id = resp.id
        outcome_repub_msg.spec = resp.wrench
        outcome_repub_msg.result = resp.result
        self.outcome_repub.publish(outcome_repub_msg)

        return resp
                                                        
def main():
    rospy.init_node('fts_detector_server')
    fts_detector = FTSDetector()
    rospy.spin()


if __name__ == '__main__':
    main()
