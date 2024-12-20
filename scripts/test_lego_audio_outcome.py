#!/usr/bin/env python3

import rospy
import os
import numpy as np

from robot_controller.yk_controller import YaskawaRobotController
from gripper_controller.lego_gripper_controller import LegoGripperController

from autolab_core import RigidTransform

from skill.lego_skills import PlaceLego, PickLego
from skill.common_skills import ResetJoints, MoveDownToContact, PullUp, MoveToAboveCorrectPose, MoveToAbovePerturbPose
from outcome.outcome import send_start_fts_outcome_request, send_end_fts_outcome_request, send_start_vision_outcome_request, send_end_vision_outcome_request, send_audio_outcome_request
from std_msgs.msg import Int16, String
from data_recorder.rosbag_data_recorder import RosbagDataRecorder
from test.check_ros_topics import check_ros_topics, check_ros_services

import yaml

STUD_WIDTH = 0.008 # mm
LEGO_BLOCK_HEIGHT=0.0096 #z
# I'm reporting length and width in the nominal/patterened spacing between studs
#  not the measured length and width with clearance added
LEGO_BLOCK_WIDTH=1*STUD_WIDTH #x    #0.0158 (with clearance)
LEGO_BLOCK_LENGTH=2*STUD_WIDTH #y   #0.0318 (with clearance)

def determine_next_pose(T_lego_world, block_x_loc, block_y_loc):

    registered_locations = []

    for key in T_lego_world.keys():
        registered_locations.append([int(k) for k in key.split(',')])
    registered_locations = np.array(registered_locations)
    #print(registered_locations)
    print([block_x_loc, block_y_loc])
    #print(registered_locations - np.array([block_x_loc, block_y_loc]))
    dist = np.linalg.norm(registered_locations - np.array([block_x_loc, block_y_loc]), axis=-1)
    #print(dist)
    index = np.argmin(dist)
    #print(index)
    closest_tf_loc = registered_locations[index]
    #print(closest_tf_loc)
    diff = [block_x_loc, block_y_loc] - closest_tf_loc
    #print(diff)
    closest_tf = T_lego_world[str(closest_tf_loc[0])+','+str(closest_tf_loc[1])]

    place_perturb_pose = RigidTransform(translation=[diff[0]*STUD_WIDTH, diff[1]*STUD_WIDTH, 0.0],
                                         from_frame='lego', to_frame='lego')

    tmp_T_lego_world = closest_tf * place_perturb_pose
    return tmp_T_lego_world

def run():
    # Start Node
    rospy.init_node("test_lego_audio_outcome")

    # Messaging Namespace
    namespace = rospy.get_param("test_lego_audio_outcome/namespace")
    root_pwd = rospy.get_param("test_lego_audio_outcome/root_pwd")
    data_path = rospy.get_param("test_lego_audio_outcome/data_dir")
    yaml_file = rospy.get_param("test_lego_audio_outcome/config")
    num_trials = rospy.get_param("test_lego_audio_outcome/num_trials")
    start_num = rospy.get_param("test_lego_audio_outcome/start_num")
    block_type = rospy.get_param("test_lego_audio_outcome/block_type")
    volume = rospy.get_param("test_lego_audio_outcome/volume")
    velocity_scale = rospy.get_param("test_lego_audio_outcome/velocity_scale")
    demo = rospy.get_param("test_lego_audio_outcome/demo")
    use_audio_terminator = rospy.get_param("test_lego_audio_outcome/use_audio_terminator")
    randomize_placement = rospy.get_param("test_lego_audio_outcome/randomize_placement")
    verbose = rospy.get_param("test_lego_audio_outcome/verbose")

    with open(root_pwd+'/config/'+yaml_file) as stream:
        try:
            config = yaml.safe_load(stream)
        except yaml.YAMLError as error:
            print(error)

    for key in config.keys():
        if isinstance(config[key], dict):
            if 'namespace' in config[key]:
                config[key]['namespace'] = namespace
            if 'topic_name' in config[key]:
                config[key]['topic_name'] = config[key]['topic_name'].replace("namespace", namespace)
        if isinstance(config[key], list):
            for i in range(len(config[key])):
                if isinstance(config[key][i], str) and 'namespace' in config[key][i]:
                    config[key][i] = config[key][i].replace("namespace", namespace)

    data_type = ''
    if demo:
        data_type = 'demo'
    else:
        data_type = 'test'

    num_correct_predictions = 0
    num_trials_completed = 0

    data_dir = data_path+'/lego/volume_'+str(volume)+'/'+block_type+'/'+data_type+'_vel_'+str(velocity_scale)+'/'

    data_dir_path_list = data_dir.split('/')
    combined_path = ''
    for data_dir_path_item in data_dir_path_list:
        combined_path += '/' + data_dir_path_item
        if not os.path.exists(combined_path):
            os.mkdir(combined_path)   

    if str(velocity_scale) == '0.01':
        move_down_velocity_scaling = 0.01
    elif str(velocity_scale) == '0.02':
        move_down_velocity_scaling = 0.02
    elif str(velocity_scale) == '0.05':
        move_down_velocity_scaling = 0.05
    elif str(velocity_scale) == 'random':
        move_down_velocity_scaling = -1

    audio_outcome_config = config['audio_detector'].copy() 
    audio_outcome_config['model_path'] = root_pwd+config['model_dir']+'audio_outcome_lego.pt'
    audio_recovery_config = config['audio_detector'].copy()
    audio_recovery_config['model_path'] = root_pwd+config['model_dir']+'audio_recovery_lego.pt'

    # Instantiate robot controller for Yaskawa API
    robot_commander = YaskawaRobotController(namespace)
    gripper_controller = LegoGripperController(namespace)
    expected_result_pub = rospy.Publisher(f'/{namespace}/expected_outcome_int', Int16, queue_size=10)
    logger_pub = rospy.Publisher(f'/{namespace}/audio_detector_logger', String, queue_size=10)

    # Load End-Effector Kinematics
    T_lego_ee = RigidTransform.load(root_pwd+config['transforms_dir']+config['lego_ee_tf'])

    # Load Lego block registration pose 
    T_lego_world = {}

    for tf in os.listdir(root_pwd+config['transforms_dir']+config['lego_world_tf']):
        x_loc = tf.split('_')[2]
        y_loc = tf.split('_')[3][:-3]
        T_lego_world[x_loc+','+y_loc] = RigidTransform.load(root_pwd+config['transforms_dir']+config['lego_world_tf']+tf)

    if randomize_placement:
        block_x_loc = np.random.randint(config['block_x_range'][0], config['block_x_range'][1])
        block_y_loc = np.random.randint(config['block_y_range'][0], config['block_y_range'][1])

        tmp_T_lego_world = determine_next_pose(T_lego_world, block_x_loc, block_y_loc)

    ### Skill Routine ###
    params = {'T_lego_ee': T_lego_ee, 
              'verbose': verbose}
    
    common_params = {'T_tcp_ee': T_lego_ee, 
                     'verbose': verbose}

    move_to_above_correct_pose_skill = MoveToAboveCorrectPose(robot_commander, gripper_controller, namespace, common_params)
    move_to_above_perturb_pose_skill = MoveToAbovePerturbPose(robot_commander, gripper_controller, namespace, common_params)
    pull_up_skill = PullUp(robot_commander, gripper_controller, namespace, params)
    move_down_to_contact_skill = MoveDownToContact(robot_commander, gripper_controller, namespace, params)
    place_lego_skill = PlaceLego(robot_commander, gripper_controller, namespace, params)
    pick_lego_skill = PickLego(robot_commander, gripper_controller, namespace, params)
    reset_joints_skill = ResetJoints(robot_commander, gripper_controller, namespace, params)
    data_recorder = RosbagDataRecorder()

    topics = []

    for topic in config['rosbag_data_recorder']['topics']:
        if 'namespace' in topic:
            topic = topic.replace("namespace", namespace)
        topics.append(topic)

    recording_params = {
        'topics': topics
    }

    for trial_num in range(start_num, start_num+num_trials):
        # Load temporary lego block registration pose 
        tmp_T_lego_world = RigidTransform.load(root_pwd+config['transforms_dir']+config['tmp_lego_world_tf'])

        x_perturb = np.random.uniform(config['x_range'][0], config['x_range'][1])
        y_perturb = np.random.uniform(config['y_range'][0], config['y_range'][1])
        theta_perturb = np.random.uniform(config['theta_range'][0], config['theta_range'][1])

        if move_down_velocity_scaling == -0.1:
            move_down_velocity_scaling = np.random.uniform(0.01, 0.1)

        expected_result_pub.publish(0)

        all_ros_topics_running = check_ros_topics(topics)
        all_ros_services_running = check_ros_services(config['ros_services'])

        if not (all_ros_topics_running and all_ros_services_running):
            print('Stopping Testing Loop.')
            break

        # 1. Begin rosbag recording
        rosbag_name = f"trial_{trial_num}-p_{x_perturb:0.4f}_{y_perturb:0.4f}_{theta_perturb:0.4f}_{move_down_velocity_scaling:0.2f}.bag"
        rosbag_path = os.path.join(data_dir, rosbag_name)

        data_recorder.start_recording(rosbag_path, recording_params)

        rospy.sleep(1)

        # 2. Determine Skill Parameters
        move_to_above_perturb_pose_params = {
            'T_tcp_world': tmp_T_lego_world,
            'place_perturbation': [x_perturb, y_perturb, theta_perturb]
        }
        move_to_above_perturb_pose_params.update(config['skill_params']['move_to_above_perturb_pose'])

        move_to_above_correct_pose_params = {
            'T_tcp_world': tmp_T_lego_world,
        }
        move_to_above_correct_pose_params.update(config['skill_params']['move_to_above_correct_pose'])

        move_down_params = {
            'velocity_scaling': move_down_velocity_scaling
        }
        move_down_params.update(config['skill_params']['move_down'])

        execution_params = {
            'skill_step_delay': 2.0
        }

        terminals = move_to_above_perturb_pose_skill.execute_skill(execution_params, move_to_above_perturb_pose_params)

        start_fts_outcome = send_start_fts_outcome_request(config['fts_detector'])
        outcomes = send_start_vision_outcome_request(config['lego_detector'])

        if outcomes['starting_top'] == 1:
            skill_type = "place"
        elif outcomes['starting_top'] == 0:
            skill_type = "pick"
        else:
            print("Error in skill type. Skipping trial")
            data_recorder.stop_recording()

            rospy.sleep(1)

            labeled_rosbag_path = rosbag_path.split(".bag")[0] + f"_vision_error.bag"
            os.rename(rosbag_path, labeled_rosbag_path)

            terminals = reset_joints_skill.execute_skill(None)
            break

        terminals = move_down_to_contact_skill.execute_skill(execution_params, move_down_params)

        audio_outcomes = send_audio_outcome_request(audio_outcome_config, terminals[0].stamp)

        print(audio_outcomes['success'])

        terminals = pull_up_skill.execute_skill(execution_params, config['skill_params']['pull_up'])

        outcomes = send_end_fts_outcome_request(config['fts_detector'])

        if outcomes['success'] == audio_outcomes['success']:
            num_correct_predictions += 1
        num_trials_completed += 1
        logger_string = str(num_correct_predictions) + '/' + str(num_trials_completed)
        logger_pub.publish(logger_string)

        if (demo and audio_outcomes['success'] == False) or (not demo and outcomes['success'] == False):
            terminals = move_to_above_correct_pose_skill.execute_skill(execution_params, move_to_above_correct_pose_params)

            expected_result_pub.publish(1)

            audio_recovery = send_audio_outcome_request(audio_recovery_config, terminals[0].stamp)

            recovery_action_normalized = audio_recovery['action']
            recovery_action = np.zeros(3)
            recovery_action[0] = (recovery_action_normalized[0] * (config['x_range'][1] - config['x_range'][0])) + config['x_range'][0]
            recovery_action[1] = (recovery_action_normalized[1] * (config['y_range'][1] - config['y_range'][0])) + config['y_range'][0]
            recovery_action[2] = (recovery_action_normalized[2] * (config['theta_range'][1] - config['theta_range'][0])) + config['theta_range'][0] 

            print(f"Ground truth perturb values: {x_perturb}, {y_perturb}, {theta_perturb}")
            print(f"Predicted perturb values: {recovery_action[0]}, {recovery_action[1]}, {recovery_action[2]}")

            start_fts_outcome = send_start_fts_outcome_request(config['fts_detector'])
            outcomes = send_start_vision_outcome_request(config['lego_detector'])

            if outcomes['starting_top'] == 1:
                skill_type = "place"
            elif outcomes['starting_top'] == 0:
                skill_type = "pick"
            else:
                print("Error in skill type. Skipping trial")
                data_recorder.stop_recording()

                rospy.sleep(1)

                labeled_rosbag_path = rosbag_path.split(".bag")[0] + f"_vision_error.bag"
                os.rename(rosbag_path, labeled_rosbag_path)

                terminals = reset_joints_skill.execute_skill(None)
                break

            terminals = move_down_to_contact_skill.execute_skill(execution_params, move_down_params)

            audio_outcomes = send_audio_outcome_request(audio_outcome_config, terminals[0].stamp)

            print(audio_outcomes['success'])

            terminals = pull_up_skill.execute_skill(execution_params, config['skill_params']['pull_up'])

            outcomes = send_end_fts_outcome_request(config['fts_detector'])

            if outcomes['success'] == audio_outcomes['success']:
                num_correct_predictions += 1
            num_trials_completed += 1
            logger_string = str(num_correct_predictions) + '/' + str(num_trials_completed)
            logger_pub.publish(logger_string)

        if (demo and audio_outcomes['success'] == False) or (not demo and outcomes['success'] == False):
            print("Failed to pull up lego. Skipping trial")
            data_recorder.stop_recording()

            rospy.sleep(1)

            labeled_rosbag_path = rosbag_path.split(".bag")[0] + f"_connection_failure.bag"
            os.rename(rosbag_path, labeled_rosbag_path)

            if skill_type == 'place' and randomize_placement:
                block_x_loc = np.random.randint(config['block_x_range'][0], config['block_x_range'][1])
                block_y_loc = np.random.randint(config['block_y_range'][0], config['block_y_range'][1])

                tmp_T_lego_world = determine_next_pose(T_lego_world, block_x_loc, block_y_loc)

                tmp_T_lego_world.save(root_pwd+config['transforms_dir']+config['tmp_lego_world_tf'])

            terminals = reset_joints_skill.execute_skill(None)
            continue
        #else:
            #terminals = move_down_to_contact_skill.execute_skill(execution_params, move_down_to_reconnect_params)

        if skill_type == "place":
            terminals = place_lego_skill.execute_skill(execution_params, config['skill_params']['place_lego'])
        elif skill_type == "pick":
            terminals = pick_lego_skill.execute_skill(execution_params, config['skill_params']['pick_lego'])

        outcomes = send_end_vision_outcome_request(config['lego_detector'])

        # 3. End rosbag recording
        data_recorder.stop_recording()

        expected_result_pub.publish(2)

        rospy.sleep(1)

        if outcomes['starting_top'] + outcomes['starting_bottom'] == outcomes['ending_top'] + outcomes['ending_bottom']:
            if outcomes['success']:
                labeled_rosbag_path = rosbag_path.split(".bag")[0] + "_" + skill_type + "_success.bag"
                os.rename(rosbag_path, labeled_rosbag_path)
            else:
                labeled_rosbag_path = rosbag_path.split(".bag")[0] + "_" + skill_type + "_failure.bag"
                os.rename(rosbag_path, labeled_rosbag_path)
        else:
            labeled_rosbag_path = rosbag_path.split(".bag")[0] + "_" + skill_type + "_error.bag"
            os.rename(rosbag_path, labeled_rosbag_path)
            break
        
        terminals = reset_joints_skill.execute_skill(None)

        if skill_type == "pick" and outcomes['success'] and randomize_placement:
            block_x_loc = np.random.randint(config['block_x_range'][0], config['block_x_range'][1])
            block_y_loc = np.random.randint(config['block_y_range'][0], config['block_y_range'][1])

            tmp_T_lego_world = determine_next_pose(T_lego_world, block_x_loc, block_y_loc)

            tmp_T_lego_world.save(root_pwd+config['transforms_dir']+config['tmp_lego_world_tf'])

if __name__ == "__main__":
    run()