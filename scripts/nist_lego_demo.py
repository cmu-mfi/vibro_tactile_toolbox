#!/usr/bin/env python3

import rospy
import os
import numpy as np

from robot_controller.yk_controller import YaskawaRobotController
from gripper_controller.lego_gripper_controller import LegoGripperController

from autolab_core import RigidTransform

from skill.lego_skills import *
from skill.util_skills import GoHomeSkill
from outcome.outcome import *
from std_msgs.msg import Int16, String
from data_recorder.rosbag_data_recorder import RosbagDataRecorder

import yaml

STUD_WIDTH = 0.008 # mm
LEGO_BLOCK_HEIGHT=0.0096 #z
# I'm reporting length and width in the nominal/patterened spacing between studs
#  not the measured length and width with clearance added
LEGO_BLOCK_WIDTH=1*STUD_WIDTH #x    #0.0158 (with clearance)
LEGO_BLOCK_LENGTH=2*STUD_WIDTH #y   #0.0318 (with clearance)

def run():
    # Start Node
    rospy.init_node("nist_lego_demo")

    # Messaging Namespace
    namespace = rospy.get_param("nist_lego_demo/namespace")
    root_pwd = rospy.get_param("nist_lego_demo/root_pwd")
    yaml_file = rospy.get_param("nist_lego_demo/config")
    num_trials = rospy.get_param("nist_lego_demo/num_trials")
    start_num = rospy.get_param("nist_lego_demo/start_num")
    block_type = rospy.get_param("nist_lego_demo/block_type")
    volume = rospy.get_param("nist_lego_demo/volume")
    velocity_scale = rospy.get_param("nist_lego_demo/velocity_scale")
    demo = rospy.get_param("nist_lego_demo/demo")
    use_audio_terminator = rospy.get_param("nist_lego_demo/use_audio_terminator")
    verbose = rospy.get_param("nist_lego_demo/verbose")

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

    data_type = ''
    if demo:
        data_type = 'demo'
    else:
        data_type = 'test'

    num_correct_predictions = 0
    num_trials_completed = 0

    data_dir = config['data_dir']+'volume_'+str(volume)+'/'+block_type+'/'+data_type+'_vel_'+str(velocity_scale)+'/'

    if not os.path.exists(config['data_dir']+'volume_'+str(volume)):
        os.mkdir(config['data_dir']+'volume_'+str(volume))
    if not os.path.exists(config['data_dir']+'volume_'+str(volume)+'/'+block_type):
        os.mkdir(config['data_dir']+'volume_'+str(volume)+'/'+block_type)
    if not os.path.exists(config['data_dir']+'volume_'+str(volume)+'/'+block_type+'/'+data_type+'_vel_'+str(velocity_scale)):
        os.mkdir(config['data_dir']+'volume_'+str(volume)+'/'+block_type+'/'+data_type+'_vel_'+str(velocity_scale))    

    if str(velocity_scale) == '0.01':
        move_down_velocity_scaling = 0.01
    elif str(velocity_scale) == '0.02':
        move_down_velocity_scaling = 0.02
    elif str(velocity_scale) == '0.05':
        move_down_velocity_scaling = 0.05
    elif str(velocity_scale) == 'random':
        move_down_velocity_scaling = -1

    outcome_config = config['audio_detector'].copy() 
    outcome_config['model_path'] = root_pwd+config['model_dir']+'audio_outcome_lego.pt'
    recovery_config = config['audio_detector'].copy()
    recovery_config['model_path'] = root_pwd+config['model_dir']+'audio_recovery_lego.pt'

    # Instantiate robot controller for Yaskawa API
    robot_commander = YaskawaRobotController(namespace)
    gripper_controller = LegoGripperController(namespace)
    expected_result_pub = rospy.Publisher(f'/{namespace}/expected_outcome_int', Int16, queue_size=10)
    logger_pub = rospy.Publisher(f'/{namespace}/audio_detector_logger', String, queue_size=10)

    # Load End-Effector Kinematics
    T_lego_ee = RigidTransform.load(root_pwd+config['transforms_dir']+config['lego_ee_tf'])

    ### Skill Routine ###
    params = {'T_lego_ee': T_lego_ee, 
              'verbose': verbose}

    move_to_above_lego_pose_skill = MoveToAboveLegoPose(robot_commander, gripper_controller, namespace, params)
    move_to_above_perturb_lego_skill = MoveToAbovePerturbLegoPose(robot_commander, gripper_controller, namespace, params)
    pull_up_skill = PullUp(robot_commander, gripper_controller, namespace, params)
    move_down_skill = MoveDown(robot_commander, gripper_controller, namespace, params)
    place_lego_skill = PlaceLego(robot_commander, gripper_controller, namespace, params)
    pick_lego_skill = PickLego(robot_commander, gripper_controller, namespace, params)
    home_skill = GoHomeSkill(robot_commander, gripper_controller, namespace, params)
    data_recorder = RosbagDataRecorder()

    topics = []

    for topic in config['rosbag_data_recorder']['topics']:
        if 'namespace' in topic:
            topic = topic.replace("namespace", namespace)
        topics.append(topic)

    recording_params = {
        'topics': topics
    }

    tmp_T_lego_world = RigidTransform.load(root_pwd+config['transforms_dir']+'T_lego_world/lego_world_-9_4.tf')

    tmp_T_lego_world.save(root_pwd+config['transforms_dir']+config['tmp_lego_world_tf'])

    for trial_num in range(start_num, start_num+num_trials):
        # Load temporary lego block registration pose 
        tmp_T_lego_world = RigidTransform.load(root_pwd+config['transforms_dir']+config['tmp_lego_world_tf'])

        x_perturb = np.random.uniform(config['x_range'][0], config['x_range'][1])
        y_perturb = np.random.uniform(config['y_range'][0], config['y_range'][1])
        theta_perturb = np.random.uniform(config['theta_range'][0], config['theta_range'][1])
        print(f"Ground truth perturb values: {x_perturb}, {y_perturb}, {theta_perturb}")

        if move_down_velocity_scaling == -0.1:
            move_down_velocity_scaling = np.random.uniform(0.01, 0.1)

        expected_result_pub.publish(0)

        # 1. Begin rosbag recording
        rosbag_name = f"trial_{trial_num}-p_{x_perturb:0.4f}_{y_perturb:0.4f}_{theta_perturb:0.4f}_{move_down_velocity_scaling:0.2f}.bag"
        rosbag_path = os.path.join(data_dir, rosbag_name)

        data_recorder.start_recording(rosbag_path, recording_params)

        rospy.sleep(1)

        # 2. Determine Skill Parameters
        move_to_above_perturb_lego_params = {
            'T_lego_world': tmp_T_lego_world,
            'approach_height_offset': config['approach_height'],
            'place_perturbation': [x_perturb, y_perturb, theta_perturb]
        }

        move_to_above_lego_params = {
            'T_lego_world': tmp_T_lego_world,
            'approach_height_offset': config['approach_height'],
        }

        pull_up_params = {
            'lift_height_offset': config['lift_height'],
            'velocity_scaling': config['pull_up_velocity_scaling']
        }

        move_down_params = {
            'height_offset': config['approach_height'],
            'velocity_scaling': move_down_velocity_scaling
        }
        if use_audio_terminator:
            move_down_params['model_path'] = root_pwd+config['model_dir']+'audio_terminator_lego.pt'

        execution_params = {
            'skill_step_delay': 2.0
        }

        place_lego_params = {
            'place_rotation': config['place_rotation'],
            'lift_height_offset': config['approach_height'],
        }

        pick_lego_params = {
            'pick_rotation': config['pick_rotation'],
            'lift_height_offset': config['approach_height'],
        }

        terminals = move_to_above_perturb_lego_skill.execute_skill(execution_params, move_to_above_perturb_lego_params)

        start_fts_outcome = send_start_fts_outcome_request(config['fts_detector'])
        outcomes = send_start_vision_outcome_request(config['lego_detector'])

        if outcomes['starting_top'] == 0 and outcomes['starting_bottom'] > 0:
            skill_type = "pick"
        elif outcomes['starting_top'] > 0:
            skill_type = "place"
        else:
            print("Error in skill type. Skipping trial")
            data_recorder.stop_recording()

            rospy.sleep(1)

            labeled_rosbag_path = rosbag_path.split(".bag")[0] + f"_vision_error.bag"
            os.rename(rosbag_path, labeled_rosbag_path)

            terminals = home_skill.execute_skill(None)
            break

        terminals = move_down_skill.execute_skill(execution_params, move_down_params)

        audio_outcomes = send_audio_outcome_request(outcome_config, terminals[0].stamp)

        print(audio_outcomes['success'])

        terminals = pull_up_skill.execute_skill(execution_params, pull_up_params)

        outcomes = send_end_fts_outcome_request(config['fts_detector'])

        if outcomes['success'] == audio_outcomes['success']:
            num_correct_predictions += 1
        num_trials_completed += 1
        logger_string = str(num_correct_predictions) + '/' + str(num_trials_completed)
        logger_pub.publish(logger_string)

        if (demo and audio_outcomes['success'] == False) or (not demo and outcomes['success'] == False):
            terminals = move_to_above_lego_pose_skill.execute_skill(execution_params, move_to_above_lego_params)

            expected_result_pub.publish(1)

            # audio_recovery = send_audio_outcome_request(recovery_config, terminals[0].stamp)

            # print(audio_recovery['result'])

            start_fts_outcome = send_start_fts_outcome_request(config['fts_detector'])
            outcomes = send_start_vision_outcome_request(config['lego_detector'])

            if outcomes['starting_top'] == 0 and outcomes['starting_bottom'] > 0:
                skill_type = "pick"
            elif outcomes['starting_top'] > 0:
                skill_type = "place"
            else:
                print("Error in skill type. Skipping trial")
                data_recorder.stop_recording()

                rospy.sleep(1)

                labeled_rosbag_path = rosbag_path.split(".bag")[0] + f"_vision_error.bag"
                os.rename(rosbag_path, labeled_rosbag_path)

                terminals = home_skill.execute_skill(None)
                break

            terminals = move_down_skill.execute_skill(execution_params, move_down_params)

            audio_outcomes = send_audio_outcome_request(outcome_config, terminals[0].stamp)

            print(audio_outcomes['success'])

            terminals = pull_up_skill.execute_skill(execution_params, pull_up_params)

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

            if skill_type == 'place':
                tmp_T_lego_world = RigidTransform.load(root_pwd+config['transforms_dir']+'T_lego_world/lego_world_-9_4.tf')

            terminals = home_skill.execute_skill(None)
            continue
        #else:
            #terminals = move_down_skill.execute_skill(execution_params, move_down_to_reconnect_params)

        if skill_type == "place":
            terminals = place_lego_skill.execute_skill(execution_params, place_lego_params)
        elif skill_type == "pick":
            terminals = pick_lego_skill.execute_skill(execution_params, pick_lego_params)

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
        
        terminals = home_skill.execute_skill(None)

        if outcomes['success']:
            tmp_T_lego_world = RigidTransform.load(root_pwd+config['transforms_dir']+'T_lego_world/lego_world_-6_8.tf')

            tmp_T_lego_world.save(root_pwd+config['transforms_dir']+config['tmp_lego_world_tf'])
            block_type = "4x1"
            data_dir = config['data_dir']+'volume_'+str(volume)+'/'+block_type+'/'+data_type+'_vel_'+str(velocity_scale)+'/'

if __name__ == "__main__":
    run()