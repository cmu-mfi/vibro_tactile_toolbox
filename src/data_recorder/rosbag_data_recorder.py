import rospy
import subprocess

from data_recorder.base_data_recorder import BaseDataRecorder

class RosbagDataRecorder(BaseDataRecorder):

    def __init__(self):
        self.recording_pid = None
        self.start_time = rospy.Time.now()

    def start_recording(self, file_name, params):
        topics = params['topics']
        command = f"exec rosbag record -O {file_name} " 

        for topic in topics:
            command += topic + " "
        print(command)
        print(f"Starting ROSbag recording ...")
        self.start_time = rospy.Time.now()
        self.recording_pid = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True)
        return self.recording_pid

    def stop_recording(self):
        self.recording_pid.terminate()
        print(f"Ending ROSbag recording.")
        end_time = rospy.Time.now()
        duration = end_time - self.start_time
        print(f"ROSbag duration: {duration.to_sec():0.2f}")
