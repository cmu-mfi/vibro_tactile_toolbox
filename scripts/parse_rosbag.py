import os
import argparse
import rosbag
import cv2
import numpy as np
from cv_bridge import CvBridge
import pickle

import soundfile as sf
from sounddevice_ros.msg import AudioInfo, AudioData

from pathlib import Path

namespace = "yk_creator"
AUDIO_TOPIC = '/audio'
CAMERA_1_COLOR_TOPIC = '/camera/color/image_raw'
CAMERA_2_COLOR_TOPIC = '/side_camera/color/image_cropped'
FTS_TOPIC = '/fts'
JOINT_STATE_TOPIC = f'/{namespace}/joint_states'
SKILL_TERMINATION_TOPIC = f'/{namespace}/terminator/skill_termination_signal'
LEGO_OUTCOME_TOPIC = f'/outcome/lego_detector'
FTS_OUTCOME_TOPIC = f'/outcome/fts_detector'

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
    print(f"formatting: [t, fx, fy, fz, tx, ty, tz]")

    if use_pkl:
        pickle.dump(fts_data, filepath)
    else:
        np.save(filepath, fts_data)

def save_joint_states(bag, save_dir, filename, joint_state_topics=[], use_pkl=False):
    '''    
    Save force torque data into a .npy or .pkl file 

    save_dir: Path to dir where data is saved
    topics: List of topics for which FTS data should be saved. [FTS_TOPIC] is sufficent.
    Return: None
    '''
    joint_state_data = []
    # [t, p1, p2, p3, p4, p5, p6, v1, v2, v3, v4, v5, v6, u1, u2, u3, u4, u5, u6]
    filepath = os.path.join(save_dir, filename)

    joint_names = None
    for topic, msg, t in bag.read_messages(topics=joint_state_topics):
        if joint_names is None:
            joint_names = msg.name
        t = [msg.header.stamp.to_sec()]
        pos = list(msg.position)
        vel = list(msg.velocity)
        eff = list(msg.effort)
        if len(eff) != len(pos):
            eff = [0] * len(pos)

        joint_state_data.append(t + pos + vel + eff)

    joint_state_data = np.array(joint_state_data, dtype=float).T
    # shift time to 0 to tf [s]
    joint_state_data[0, :] -= joint_state_data[0, 0]

    print(f"{joint_state_topics[0]}")
    print(f"joint names: {joint_names}")
    print(f"{joint_state_data.shape}, t0: {joint_state_data[0, 0]}, tf: {joint_state_data[0, -1]}")
    print(f"formatting: [t, p1, p2, p3, p4, p5, p6, v1, v2, v3, v4, v5, v6, u1, u2, u3, u4, u5, u6]")

    if use_pkl:
        pickle.dump(joint_state_data, filepath)
    else:
        np.save(filepath, joint_state_data)

def save_termination_signals(bag, save_dir, filename, termination_topics=[]):
    '''    
    Save termination messsages as a .txt file

    save_dir: Path to dir where data is saved
    topics: List of topics for which FTS data should be saved. [FTS_TOPIC] is sufficent.
    Return: None
    '''    

    filepath = os.path.join(save_dir, filename)
    text_file = os.path.join(save_dir, "termination_signals.txt")
    with open(text_file, 'w') as f:
        f.write(f"# fts_detector.py outcomes\n")
        f.write(f"# file: '{bag.filename}'\n")
        f.write(f"# timestamp termination_cause\n")
        for topic, msg, t in bag.read_messages(topics=termination_topics):
            t_trial = t.to_sec() - bag.get_start_time()
            print(f"Termination message at time {t_trial}")
            termination_cause = msg.cause
            f.write(f"{t_trial}, {termination_cause}\n")

def save_outcomes(bag, save_dir, filenames=[], outcome_topics=[]):
    '''    
    Save outcome messsages as a .txt file with annotated images to lego_detections/
    Note these are NOT the direct service calls, but a republished outcome message

    save_dir: Path to dir where data is saved
    topics: List of topics for which FTS data should be saved. [FTS_TOPIC] is sufficent.
    Return: None
    '''   
    bridge = CvBridge()

    outcome_data = []

    lego_detections_dir = os.path.join(save_dir, "lego_detections")
    if not os.path.exists(lego_detections_dir):
        os.mkdir(lego_detections_dir)

    for i, outcome_topic in enumerate(outcome_topics):
        if "fts" in outcome_topic:
            text_file = os.path.join(save_dir, "fts_outcomes.txt")
            with open(text_file, 'w') as f:
                f.write(f"# fts_detector.py outcomes\n")
                f.write(f"# file: '{bag.filename}'\n")
                f.write(f"# timestamp ???\n")
                for topic, msg, t in bag.read_messages(topics=outcome_topic):
                    t_trial = t.to_sec() - bag.get_start_time()
                    print(f"fts_detector outcome message at time {t_trial}")
                    termination_cause = "TODO: MAKE FTS_OUTCOME MSG MORE INFORMATIVE" # msg.outcome
                    f.write(f"{t_trial}, {termination_cause}\n")

        elif "lego" in outcome_topic:
            text_file = os.path.join(save_dir, "lego_outcomes.txt")
            with open(text_file, 'w') as f:
                f.write(f"# lego_detector.py outcomes\n")
                f.write(f"# file: '{bag.filename}'\n")
                f.write(f"# timestamp filename\n")
                for topic, msg, t in bag.read_messages(topics=outcome_topic):
                    t_trial = t.to_sec() - bag.get_start_time()
                    print(f"lego_detector outcome message at time {t_trial}")
                    lego_outcome = "TODO: MAKE LEGO_OUTCOME MSG MORE INFORMATIVE" # msg.outcome
                    img_ann = bridge.imgmsg_to_cv2(msg)
                    img_name = f"{t_trial}.png" 
                    img_path = os.path.join(lego_detections_dir, img_name)
                    print(f"Saving annotated image to {img_path}")
                    cv2.imwrite(img_path, img_ann)
                    f.write(f"{t_trial}, {lego_outcome}, lego_detections/{img_name}\n")
                

        else:
            print(f"Unhandled outcome topic requested for parsing: {outcome_topic}")
            continue



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

    if args.save_joints:
        joint_state_topics = [JOINT_STATE_TOPIC]
        save_joint_states(bag, save_folder, 'joint_states', joint_state_topics)

    if args.save_termination:
        termination_topics = [SKILL_TERMINATION_TOPIC]
        save_termination_signals(bag, save_folder, 'termination_signals', termination_topics)

    if args.save_outcome:
        outcome_topics = [LEGO_OUTCOME_TOPIC,
                          FTS_OUTCOME_TOPIC]
        save_outcomes(bag, save_folder, 'outcomes', outcome_topics)

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
    parser.add_argument('--save_joints', '-j', type=bool, default=True,
                        help='True if save joint states else False. Default True.')
    parser.add_argument('--save_termination', '-t', type=bool, default=True,
                        help='True if save termination signals, else False. Default True.')
    parser.add_argument('--save_outcome', '-o', type=bool, default=True,
                        help='True if save outcome responses, else False. Default True.')
    args = parser.parse_args()

    main(args)
