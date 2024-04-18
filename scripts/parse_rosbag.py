import os
import argparse
import rosbag
import cv2
import numpy as np
from cv_bridge import CvBridge
import pickle

import soundfile as sf
from sounddevice_ros.msg import AudioInfo, AudioData

AUDIO_TOPIC = '/audio'
CAMERA_1_COLOR_TOPIC = '/camera/color/image_raw'
CAMERA_2_COLOR_TOPIC = '/side_camera/color/image_cropped'
FTS_TOPIC = '/fts'

def create_dir_if_not_exists(dir_path):
    if not os.path.exists(dir_path):
        os.makedirs(dir_path)

def get_audio_info(bag, audio_info_topic):
    audio_info = {}

    for topic, msg, t in bag.read_messages(topics=[audio_info_topic]):
        audio_info['num_channels'] = msg.num_channels
        audio_info['sample_rate'] = msg.sample_rate
        if msg.subtype:
            audio_info['subtype'] = msg.subtype
        else:
            audio_info['subtype'] = None
        break

    return audio_info


def save_audio(bag, save_dir, audio_file_name, audio_info, audio_topic):

    audio_file_path = os.path.join(save_dir, audio_file_name)

    try:
        os.remove(audio_file_path)
    except OSError:
        pass


    sound_file = sf.SoundFile(audio_file_path, mode='x', 
                              samplerate=audio_info['sample_rate'],
                              channels=audio_info['num_channels'], 
                              subtype=audio_info['subtype'])

    for topic, msg, t in bag.read_messages(topics=[audio_topic]):
        sound_file.write(np.asarray(msg.data).reshape((-1, audio_info['num_channels'])))

    sound_file.close()
        

def save_video(bag, save_dir, filenames=[], image_topics=[]):
    '''Save videos for each image topic.

    save_img_dir: Path to dir where images will be saved.
    topics: List of topics for which images should be saved.
    save_skills_separately: Save images for each skill separately in different 
        dirs.
    Return: None
    '''


    bridge = CvBridge()

    video_dict = {}

    bag_duration = bag.get_end_time() - bag.get_start_time()

    video_freq = []

    for i, image_topic in enumerate(image_topics):
        print(image_topic)
        video_freq = bag.get_message_count([image_topic]) / bag_duration
        if video_freq > 0.0:
            for topic, msg, t in bag.read_messages(topics=[image_topic]):
                video_width = msg.width
                video_height = msg.height
                print(video_width, video_height)
                video_dict[image_topic] = cv2.VideoWriter(os.path.join(save_dir,filenames[i]+'.mp4'),  
                                          cv2.VideoWriter_fourcc(*'mp4v'), 
                                          video_freq, (video_width, video_height)) 
                break

    for topic, msg, t in bag.read_messages(topics=image_topics):
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        video_dict[topic].write(frame)
        # cv2.imshow(topic, frame) 
  
        # cv2.waitKey(1) 

    for image_topic in image_topics:
        if image_topic in video_dict.keys():
            video_dict[image_topic].release()

def save_fts(bag, save_dir, filename, fts_topics=[], use_pkl=False):
    '''
    Save force torque data into a .npy or .pkl file 

    save_dir: Path to dir where data is saved
    topics: List of topics for which FTS data should be saved. [FTS_TOPIC] is sufficent.
    Return: None
    '''
    fts_data = []
    # [t, fx, fy, fz, tx, ty, tz].T
    filepath = os.path.join(save_dir, filename)

    for topic, msg, t in bag.read_messages(topics=fts_topics):
        t = msg.header.stamp.to_sec()
        fx = msg.wrench.force.x
        fy = msg.wrench.force.y
        fz = msg.wrench.force.z
        tx = msg.wrench.torque.x
        ty = msg.wrench.torque.y
        tz = msg.wrench.torque.z
        
        fts_data.append([t, fx, fy, fz, tx, ty, tz])

    fts_data = np.array(fts_data, dtype=float).T
    # shift time to 0 to tf [s]
    fts_data[0, :] -= fts_data[0, 0]

    print(f"{fts_topics[0]}")
    print(f"{fts_data.shape}, t0: {fts_data[0, 0]}, tf: {fts_data[0, -1]}")

    if use_pkl:
        pickle.dump(fts_data, filepath)
    else:
        np.save(filepath, fts_data)
    
def main(args):
    assert os.path.exists(args.bagfile), "Rosbag does not exist"

    bag = rosbag.Bag(args.bagfile)

    create_dir_if_not_exists(args.save_dir)
    short_bag_name = args.bagfile[(args.bagfile.rfind('/')+1):args.bagfile.rfind('.bag')]
    save_folder = os.path.join(args.save_dir,short_bag_name)
    create_dir_if_not_exists(save_folder)

    if args.save_video:
        img_topics = [CAMERA_1_COLOR_TOPIC, 
                      CAMERA_2_COLOR_TOPIC]
        file_names = ['wrist_camera',
                      'side_camera']
        save_video(bag, save_folder, file_names, img_topics)

    if args.save_audio:
        audio_info = get_audio_info(bag, AUDIO_TOPIC + '_info')
        save_audio_dir = os.path.join(args.save_dir, 'audio')
        save_audio(bag, save_folder, 'audio.wav', audio_info, AUDIO_TOPIC)
    
    if args.save_fts:
        fts_topics = [FTS_TOPIC]
        save_fts(bag, save_folder, 'fts', fts_topics)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Creates audio files and videos from bagfile.')
    parser.add_argument('--bagfile', '-b', type=str, required=True,
                        help='input bag file')
    parser.add_argument('--save_dir', '-d', type=str, required=True,
                        help='Path to save rosbag data such as audio and video')
    parser.add_argument('--save_audio', '-a', type=bool, default=True,
                        help='True if save audio else False. Default True.')
    parser.add_argument('--save_video', '-v', type=bool, default=True,
                        help='True if save videos else False. Default True.')
    parser.add_argument('--save_fts', '-f', type=bool, default=True,
                        help='True if save fts else False. Default True.')
    args = parser.parse_args()

    main(args)
