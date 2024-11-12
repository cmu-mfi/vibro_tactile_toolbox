#!/usr/bin/env python3

import argparse
import rospy

from data_recorder.rosbag_data_recorder import RosbagDataRecorder

def main(args):
    rospy.init_node('test_rosbag_data_recorder', anonymous=True)
    data_recorder = RosbagDataRecorder()
    topics = [f"/{args.namespace}/side_camera/color/image_cropped/compressed", 
              f"/{args.namespace}/wrist_camera/color/image_raw/compressed", 
              f"/{args.namespace}/joint_states", 
              f"/{args.namespace}/fts", 
              f"/{args.namespace}/audio", 
              f"/{args.namespace}/audio_info"]
    params = {'topics' : topics}
    data_recorder.start_recording('test.bag', params)
    rospy.sleep(5)
    data_recorder.stop_recording()

if __name__ == '__main__':
    args = argparse.ArgumentParser()
    args.add_argument('-n', '--namespace', type=str,
      help='Namespace to use')
    args = args.parse_args()

    main(args)
