import os
import argparse
import rosbag
import cv2
import numpy as np
import pickle
import json

import soundfile as sf
from sounddevice_ros.msg import AudioInfo, AudioData

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
        

def save_audio_npy(bag, save_dir, audio_file_name, audio_info, audio_topic):

    audio_np_path = os.path.join(save_dir, audio_file_name)

    audio_np_array = np.zeros((audio_info['num_channels'],0))

    for topic, msg, t in bag.read_messages(topics=[audio_topic]):
        audio_np_array = np.hstack((audio_np_array, np.asarray(msg.data).reshape((-1, audio_info['num_channels'])).transpose()))


    np.save(audio_np_path, audio_np_array)
        

def save_video(bag, save_dir, filenames=[], image_topics=[], video_latency=0.0):
    '''Save videos for each image topic.

    save_img_dir: Path to dir where images will be saved.
    topics: List of topics for which images should be saved.
    save_skills_separately: Save images for each skill separately in different 
        dirs.
    Return: None
    '''

    video_dict = {}

    bag_duration = bag.get_end_time() - bag.get_start_time()

    video_freq = []

    for i, image_topic in enumerate(image_topics):
        print(image_topic)
        video_freq = bag.get_message_count([image_topic]) / bag_duration
        if video_freq > 0.0:
            for topic, msg, t in bag.read_messages(topics=[image_topic]):
                np_arr = np.frombuffer(msg.data, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                video_width = frame.shape[1]
                video_height = frame.shape[0]
                print(video_width, video_height)
                video_dict[image_topic] = cv2.VideoWriter(os.path.join(save_dir,filenames[i]+'.mp4'),  
                                          cv2.VideoWriter_fourcc(*'mp4v'), 
                                          video_freq, (video_width, video_height)) 
                break

    timestamps = {f'{image_topics[0]}': [], f'{image_topics[1]}': []}
    last_frame = {image_topics[0]: None, image_topics[1]: None}
    skipped_frames = {image_topics[0]: 0, image_topics[1]: 0}
    for topic, msg, t in bag.read_messages(topics=image_topics):
        t = t.to_sec() - bag.get_start_time() - video_latency
        if t < 0 and topic == image_topics[1]:
            skipped_frames[topic] += 1
            continue
        timestamps[topic].append(t)
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        video_dict[topic].write(frame)

        last_frame[topic] = frame
    
    for topic in image_topics:
        for i in range(skipped_frames[topic]):
            timestamps[topic].append(timestamps[topic][-1])
            video_dict[topic].write(last_frame[topic])
        # cv2.imshow(topic, frame) 
  
        # cv2.waitKey(1) 
 
    
    print(f"/SIDE_CAMERA t0: {timestamps[image_topics[1]][0]}, tf: {timestamps[image_topics[1]][-1]}")

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

    timestamps = []
    for topic, msg, t in bag.read_messages(topics=fts_topics):
        t = t.to_sec() - bag.get_start_time()
        timestamps.append(t)
        # t = msg.header.time.to_sec()
        fx = msg.wrench.force.x
        fy = msg.wrench.force.y
        fz = msg.wrench.force.z
        tx = msg.wrench.torque.x
        ty = msg.wrench.torque.y
        tz = msg.wrench.torque.z
        
        fts_data.append([t, fx, fy, fz, tx, ty, tz])

    fts_data = np.array(fts_data, dtype=float).T

    print(f"{fts_topics[0]}")
    print(f"{fts_data.shape}")
    print(f"formatting: [t, fx, fy, fz, tx, ty, tz]")

    print(f"/FTS t0: {timestamps[0]}, tf: {timestamps[-1]}")
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
        #t = [msg.header.stamp.to_sec()]
        t = [t.to_sec() - bag.get_start_time()]
        pos = list(msg.position)
        vel = list(msg.velocity)
        eff = list(msg.effort)
        if len(eff) != len(pos):
            eff = [0] * len(pos)

        joint_state_data.append(t + pos + vel + eff)

    joint_state_data = np.array(joint_state_data, dtype=float).T

    print(f"{joint_state_topics[0]}")
    print(f"joint names: {joint_names}")
    print(f"{joint_state_data.shape}")
    print(f"formatting: [t, p1, p2, p3, p4, p5, p6, v1, v2, v3, v4, v5, v6, u1, u2, u3, u4, u5, u6]")
    print(f"/JOINTS t0: {joint_state_data[0, 0]}, tf: {joint_state_data[0, -1]}")
    if use_pkl:
        pickle.dump(joint_state_data, filepath)
    else:
        np.save(filepath, joint_state_data)


def save_skill_params(bag, save_dir, filename, skill_param_topics=[]):
    '''    
    Save skill param messsages as a .txt file

    save_dir: Path to dir where data is saved
    topics: List of topics for which skill param data should be saved. 
    Return: None
    '''    

    filepath = os.path.join(save_dir, filename)
    text_file = os.path.join(save_dir, "skill_params.txt")
    with open(text_file, 'w') as f:
        f.write(f"# skill params \n")
        f.write(f"# file: '{bag.filename}'\n")
        f.write(f"# timestamp, skill id, skill name, step_name \n")
        for topic, msg, t in bag.read_messages(topics=skill_param_topics):
            t_trial = t.to_sec() - bag.get_start_time()
            print(f"Skill param message at time {t_trial}")
            f.write(f"{t_trial}, {msg.id}, {msg.skill_name}, {msg.step_name} \n")


def save_termination_signals(bag, save_dir, filename, termination_topics=[]):
    '''    
    Save termination messsages as a .txt file

    save_dir: Path to dir where data is saved
    topics: List of topics for which termination data should be saved.
    Return: None
    '''    

    filepath = os.path.join(save_dir, filename)
    text_file = os.path.join(save_dir, "termination_signals.txt")
    with open(text_file, 'w') as f:
        f.write(f"# terminator outcomes\n")
        f.write(f"# file: '{bag.filename}'\n")
        f.write(f"# timestamp termination_cause\n")
        for topic, msg, t in bag.read_messages(topics=termination_topics):
            t_trial = t.to_sec() - bag.get_start_time()
            print(f"Termination message at time {t_trial}")
            termination_cause = msg.cause
            f.write(f"{t_trial}, {msg.id}, {termination_cause}\n")

def save_outcomes(bag, save_dir, filenames=[], outcome_topics=[], outcome_latency=0.0):
    '''    
    Save outcome messsages as a .txt file with annotated images to lego_detections/
    Note these are NOT the direct service calls, but a republished outcome message

    save_dir: Path to dir where data is saved
    topics: List of topics for which FTS data should be saved. [FTS_TOPIC] is sufficent.
    Return: None
    '''   

    outcome_data = []

    lego_detections_dir = os.path.join(save_dir, "lego_detections")
    if not os.path.exists(lego_detections_dir):
        os.mkdir(lego_detections_dir)

    for i, outcome_topic in enumerate(outcome_topics):
        if "audio" in outcome_topic:
            text_file = os.path.join(save_dir, "audio_outcomes.txt")
            with open(text_file, 'w') as f:
                f.write(f"# audio_detector.py outcomes\n")
                f.write(f"# file: '{bag.filename}'\n")
                f.write(f"# timestamp, result, success\n")
                for topic, msg, t in bag.read_messages(topics=outcome_topic):
                    t_trial = t.to_sec() - bag.get_start_time()
                    print(f"audio_detector outcome message at time {t_trial}")
                    outcome = json.loads(msg.result)
                    if 'success' in outcome.keys():
                        result = outcome['result'].strip()
                        success = outcome['success']
                        f.write(f"{t_trial}, {result}, {success}\n")
                    else:
                        recovery_action = outcome['action']
                        f.write(f"{t_trial}, {recovery_action} \n")
        elif "fts" in outcome_topic:
            text_file = os.path.join(save_dir, "fts_outcomes.txt")
            with open(text_file, 'w') as f:
                f.write(f"# fts_detector.py outcomes\n")
                f.write(f"# file: '{bag.filename}'\n")
                f.write(f"# timestamp, result, success\n")
                for topic, msg, t in bag.read_messages(topics=outcome_topic):
                    t_trial = t.to_sec() - bag.get_start_time()
                    print(f"fts_detector outcome message at time {t_trial}")
                    outcome = json.loads(msg.result)
                    result = outcome['result'].strip()
                    success = outcome['success']

                    f.write(f"{t_trial}, {result}, {success}\n")
        elif "lego" in outcome_topic:
            text_file = os.path.join(save_dir, "lego_outcomes.txt")
            with open(text_file, 'w') as f:
                f.write(f"# lego_detector.py outcomes\n")
                f.write(f"# file: '{bag.filename}'\n")
                f.write(f"# timestamp, result, success, filename\n")
                for topic, msg, t in bag.read_messages(topics=outcome_topic):
                    t_trial = t.to_sec() - bag.get_start_time()
                    print(f"lego_detector outcome message at time {t_trial}")
                    outcome = json.loads(msg.result)
                    result = outcome['result'].strip()
                    success = outcome['success']
                    img_np_arr = np.frombuffer(msg.img.data, np.uint8)
                    img_ann = cv2.imdecode(img_np_arr, cv2.IMREAD_COLOR)
                    img_name = f"{t_trial}.png" 
                    img_path = os.path.join(lego_detections_dir, img_name)
                    print(f"Saving annotated image to {img_path}")
                    cv2.imwrite(img_path, img_ann)
                    f.write(f"{t_trial}, {result}, {success}, lego_detections/{img_name}\n")
        else:
            print(f"Unhandled outcome topic requested for parsing: {outcome_topic}")
            continue


def parse(args, bagfile, save_dir):
    assert os.path.exists(bagfile), "Rosbag does not exist"

    namespace = args.namespace
    AUDIO_TOPIC = f'/{namespace}/audio'
    CAMERA_1_COLOR_TOPIC = f'/{namespace}/wrist_camera/color/image_raw/compressed'
    CAMERA_2_COLOR_TOPIC = f'/{namespace}/side_camera/color/image_cropped/compressed'
    FTS_TOPIC = f'/{namespace}/fts'
    JOINT_STATE_TOPIC = f'/{namespace}/joint_states'
    SKILL_PARAM_TOPIC = f'/{namespace}/skill/param'
    SKILL_TERMINATION_TOPIC = f'/{namespace}/terminator/skill_termination_signal'
    AUDIO_OUTCOME_TOPIC = f'/{namespace}/outcome/audio_detector'
    LEGO_OUTCOME_TOPIC = f'/{namespace}/outcome/lego_detector'
    FTS_OUTCOME_TOPIC = f'/{namespace}/outcome/fts_detector'


    bag = rosbag.Bag(bagfile)
    print(f"BAG: t0: {bag.get_start_time()}, tf: {bag.get_end_time()}")

    create_dir_if_not_exists(save_dir)
    short_bag_name = bagfile[(bagfile.rfind('/')+1):bagfile.rfind('.bag')]
    save_folder = os.path.join(save_dir,short_bag_name)
    create_dir_if_not_exists(save_folder)

    if args.save_video:
        img_topics = [CAMERA_1_COLOR_TOPIC, 
                      CAMERA_2_COLOR_TOPIC]
        file_names = ['wrist_camera',
                      'side_camera']
        save_video(bag, save_folder, file_names, img_topics, video_latency=args.video_latency)

    if args.save_audio:
        audio_info = get_audio_info(bag, AUDIO_TOPIC + '_info')
        save_audio_dir = os.path.join(save_dir, 'audio')
        save_audio(bag, save_folder, 'audio.wav', audio_info, AUDIO_TOPIC)
        save_audio_npy(bag, save_folder, 'audio.npy', audio_info, AUDIO_TOPIC)
    
    if args.save_fts:
        fts_topics = [FTS_TOPIC]
        save_fts(bag, save_folder, 'fts', fts_topics)

    if args.save_joints:
        joint_state_topics = [JOINT_STATE_TOPIC]
        save_joint_states(bag, save_folder, 'joint_states', joint_state_topics)

    if args.save_skill_params:
        skill_param_topics = [SKILL_PARAM_TOPIC]
        save_skill_params(bag, save_folder, 'skill_params', skill_param_topics)

    if args.save_termination:
        termination_topics = [SKILL_TERMINATION_TOPIC]
        save_termination_signals(bag, save_folder, 'termination_signals', termination_topics)

    if args.save_outcome:
        outcome_topics = [AUDIO_OUTCOME_TOPIC,
                          LEGO_OUTCOME_TOPIC,
                          FTS_OUTCOME_TOPIC]
        save_outcomes(bag, save_folder, 'outcomes', outcome_topics, outcome_latency=args.lego_detector_latency)


def main(args):
    if args.walk_dir:
        list_dir = os.listdir(args.walk_dir)
        for bagfile in list_dir:
            if '.bag' in bagfile:
                if bagfile[:-4] in list_dir and args.skip:
                    print(f'Bag File: {bagfile} has been already processed. Skipping this bag')
                else:
                    parse(args, os.path.join(args.walk_dir, bagfile), args.save_dir)
    elif args.bagfile:
        parse(args, args.bagfile, args.save_dir)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Creates audio files and videos from bagfile.')

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--bagfile', '-b', type=str, help='input bag file')
    group.add_argument('--walk_dir', '-w', type=str, help="Root directory to walk through and parse all folders")
   
    parser.add_argument('--save_dir', '-d', type=str, required=True,
                        help='Path to save rosbag data such as audio and video')
    parser.add_argument('--namespace', '-n', type=str, required=True,
                        help='Robot namespace')
    parser.add_argument('--skip', '-s', type=bool, default=True,
                        help='True if skip rosbags already processed. Default True.')
    parser.add_argument('--save_audio', '-a', type=bool, default=True,
                        help='True if save audio else False. Default True.')
    parser.add_argument('--save_video', '-v', type=bool, default=True,
                        help='True if save videos else False. Default True.')
    parser.add_argument('--save_fts', '-f', type=bool, default=True,
                        help='True if save fts else False. Default True.')
    parser.add_argument('--save_joints', '-j', type=bool, default=True,
                        help='True if save joint states else False. Default True.')
    parser.add_argument('--save_skill_params', '-p', type=bool, default=True,
                        help='True if save skill params, else False. Default True.')
    parser.add_argument('--save_termination', '-t', type=bool, default=True,
                        help='True if save termination signals, else False. Default True.')
    parser.add_argument('--save_outcome', '-o', type=bool, default=True,
                        help='True if save outcome responses, else False. Default True.')
    parser.add_argument('--video_latency', type=float, default=0.0,
                        help='Specify the video latency in seconds (float). Default 0.0s')
    parser.add_argument('--lego_detector_latency', type=float, default=0.0,
                        help='Specify the lego detector latency in seconds(float). Default 0.0s')
    args = parser.parse_args()

    main(args)
