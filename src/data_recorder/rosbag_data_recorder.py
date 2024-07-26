import rospy
import subprocess
import os

from data_recorder.base_data_recorder import BaseDataRecorder

class RosbagDataRecorder(BaseDataRecorder):

    def __init__(self):
        self.recording_pid = None
        self.start_time = rospy.Time.now()

    def start_recording(self, file_name, params):
        topics = params['topics']
        command = f"exec rosbag record -b 1024 -O {file_name} __name:=bagger " 

        for topic in topics:
            command += topic + " "
        self.command = command
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

def terminate_bag_and_children(p):
    # https://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
    # For some reason bag files started failing to close gracefully
    # looking into options for fixing, but deleting the hdd1 trash seemed to do it
    ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=subprocess.PIPE)
    ps_output = ps_command.stdout.read()
    retcode = ps_command.wait()
    assert retcode == 0, "ps command returned %d" % retcode
    for pid_str in ps_output.split("\n")[:-1]:
            os.kill(int(pid_str), subprocess.signal.SIGINT)
    p.terminate()