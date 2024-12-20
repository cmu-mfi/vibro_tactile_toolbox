#!/usr/bin/env python3

import rospy
import os
import numpy as np

from robot_controller.yk_controller import YaskawaRobotController
from gripper_controller.robotiq_hande_controller import RobotiqHandEController

from autolab_core import RigidTransform

from skill.nist_skills import PickConnector, PlaceConnector, PlaceConnectorReset, ResetConnector
from skill.common_skills import ResetJoints, MoveDownToContact, PullUp, OpenGripper, MoveUp, PushDown, MoveToAboveCorrectPose, MoveToAbovePerturbPose
from outcome.outcome import send_audio_outcome_request, send_start_fts_outcome_request, send_end_fts_outcome_request
from std_msgs.msg import Int16, String
from data_recorder.rosbag_data_recorder import RosbagDataRecorder
from sklearn.metrics import confusion_matrix
from test.check_ros_topics import check_ros_topics, check_ros_services

import yaml


def run():
    # Start Node
    rospy.init_node("test_nist_audio_outcome_node")

    # Messaging Namespace
    namespace = rospy.get_param("test_nist_audio_outcome_node/namespace")
    root_pwd = rospy.get_param("test_nist_audio_outcome_node/root_pwd")
    data_path = rospy.get_param("collect_lego_audio_data/data_dir")
    yaml_file = rospy.get_param("test_nist_audio_outcome_node/config")
    num_trials = rospy.get_param("test_nist_audio_outcome_node/num_trials")
    start_num = rospy.get_param("test_nist_audio_outcome_node/start_num")
    connector_type = rospy.get_param("test_nist_audio_outcome_node/connector_type")
    volume = rospy.get_param("test_nist_audio_outcome_node/volume")
    velocity_scale = rospy.get_param("test_nist_audio_outcome_node/velocity_scale")
    reset = rospy.get_param("test_nist_audio_outcome_node/reset")
    lift = rospy.get_param("test_nist_audio_outcome_node/lift")
    demo = rospy.get_param("test_nist_audio_outcome_node/demo")
    use_audio_terminator = rospy.get_param("test_nist_audio_outcome_node/use_audio_terminator")
    verbose = rospy.get_param("test_nist_audio_outcome_node/verbose")

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

    num_correct_predictions = 0
    num_trials_completed = 0

    data_type = ''

    if demo:
        data_type = 'demo'
    else:
        data_type = 'test'

    data_dir = data_path+'/nist/volume_'+str(volume)+'/'+connector_type+'/'+data_type+'_vel_'+str(velocity_scale)+'/'

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

    # Instantiate robot controller for Yaskawa API
    robot_commander = YaskawaRobotController(namespace)
    gripper_controller = RobotiqHandEController(namespace)
    expected_result_pub = rospy.Publisher(f'/{namespace}/expected_outcome_int', Int16, queue_size=10)
    logger_pub = rospy.Publisher(f'/{namespace}/audio_detector_logger', String, queue_size=10)

    # Load End-Effector Kinematics
    T_hande_ee = RigidTransform.load(root_pwd+config['transforms_dir']+config['hande_ee_tf'])

    T_connector_world_pick = RigidTransform.load(root_pwd+config['transforms_dir']+connector_type+'/world_pick.tf')
    T_connector_world_place = RigidTransform.load(root_pwd+config['transforms_dir']+connector_type+'/world_place.tf')
       
    audio_outcome_config = config['audio_detector'].copy()
    audio_outcome_config['model_path'] = root_pwd+config['model_dir']+'audio_outcome_'+connector_type+'.pt'
    audio_recovery_config = config['audio_detector'].copy()
    audio_recovery_config['model_path'] = root_pwd+config['model_dir']+'audio_recovery_'+connector_type+'.pt'

    ### Skill Routine ###
    params = {'T_hande_ee': T_hande_ee, 
              'verbose': verbose}
    
    common_params = {'T_tcp_ee': T_hande_ee, 
                     'verbose': verbose}


    pick_connector_params = {
        'T_hande_ee': T_hande_ee, 
        'verbose': verbose,
        'T_connector_world': T_connector_world_pick
    }
    pick_connector_params.update(config['skill_params']['pick_connector'])
    place_connector_params = {
        'T_hande_ee': T_hande_ee, 
        'verbose': verbose,
        'T_connector_world': T_connector_world_pick
    }
    place_connector_params.update(config['skill_params']['place_connector'])
    
    if reset:
        T_connector_world_place_reset = RigidTransform.load(root_pwd+config['transforms_dir']+connector_type+'/world_place_reset.tf')
        T_connector_world_reset_x = RigidTransform.load(root_pwd+config['transforms_dir']+connector_type+'/world_reset_x.tf')
        T_connector_world_reset_y = RigidTransform.load(root_pwd+config['transforms_dir']+connector_type+'/world_reset_y.tf')

        place_connector_reset_params = {
            'T_hande_ee': T_hande_ee, 
            'verbose': verbose,
            'T_connector_world': T_connector_world_place_reset
        }
        place_connector_reset_params.update(config['skill_params']['place_connector_reset'])
        reset_connector_params = {
            'T_hande_ee': T_hande_ee, 
            'verbose': verbose,
            'T_connector_world_reset_x': T_connector_world_reset_x,
            'T_connector_world_reset_y': T_connector_world_reset_y
        }
        reset_connector_params.update(config['skill_params']['reset_connector'])

        place_connector_reset_skill = PlaceConnectorReset(robot_commander, gripper_controller, namespace, place_connector_reset_params)
        reset_connector_skill = ResetConnector(robot_commander, gripper_controller, namespace, reset_connector_params)

    move_to_above_correct_pose_skill = MoveToAboveCorrectPose(robot_commander, gripper_controller, namespace, common_params)
    move_to_above_perturb_pose_skill = MoveToAbovePerturbPose(robot_commander, gripper_controller, namespace, common_params)
    pull_up_skill = PullUp(robot_commander, gripper_controller, namespace, params)
    move_up_skill = MoveUp(robot_commander, gripper_controller, namespace, params)
    move_down_to_contact_skill = MoveDownToContact(robot_commander, gripper_controller, namespace, params)
    push_down_skill = PushDown(robot_commander, gripper_controller, namespace, params)
    pick_connector_skill = PickConnector(robot_commander, gripper_controller, namespace, pick_connector_params)
    place_connector_skill = PlaceConnector(robot_commander, gripper_controller, namespace, place_connector_params)

    reset_joints_skill = ResetJoints(robot_commander, gripper_controller, namespace, params)
    open_gripper_skill = OpenGripper(robot_commander, gripper_controller, namespace, params)
    data_recorder = RosbagDataRecorder()

    predicted_outcomes = []
    actual_outcomes = []

    topics = []

    for topic in config['rosbag_data_recorder']['topics']:
        if 'namespace' in topic:
            topic = topic.replace("namespace", namespace)
        topics.append(topic)

    recording_params = {
        'topics': topics
    }

    execution_params = {
        'skill_step_delay': 1.0
    }

    terminals = open_gripper_skill.execute_skill(None)
    if lift:
        terminals = move_up_skill.execute_skill(execution_params, config['skill_params']['move_up'])
    terminals = reset_joints_skill.execute_skill(None)

    if reset:
        reset_connector_skill.execute_skill(execution_params)

    expected_result_pub.publish(2)

    # Tasks to do
    for trial_num in range(start_num, start_num+num_trials):

        pick_connector_skill.execute_skill(execution_params)

        x_perturb = np.random.uniform(config['x_range'][0], config['x_range'][1])
        y_perturb = np.random.uniform(config['y_range'][0], config['y_range'][1])
        theta_perturb = np.random.uniform(config['theta_range'][0], config['theta_range'][1])
        print(f"Ground truth perturb values: {x_perturb}, {y_perturb}, {theta_perturb}")

        if move_down_velocity_scaling == -0.1:
            move_down_velocity_scaling = np.random.uniform(0.01, 0.1)

        # 2. Determine Skill Parameters
        move_to_above_perturb_pose_params = {
            'T_tcp_world': T_connector_world_place,
            'place_perturbation': [x_perturb, y_perturb, theta_perturb]
        }
        move_to_above_perturb_pose_params.update(config['skill_params']['move_to_above_perturb_pose'])

        move_to_above_correct_pose_params = {
            'T_tcp_world': T_connector_world_place,
        }
        move_to_above_correct_pose_params.update(config['skill_params']['move_to_above_correct_pose'])

        move_down_params = {
            'velocity_scaling': move_down_velocity_scaling
        }
        move_down_params.update(config['skill_params']['move_down'])

        if use_audio_terminator:
            move_down_params['model_path'] = root_pwd+config['model_dir']+'audio_terminator_'+connector_type+'.pt'

        terminals = move_to_above_perturb_pose_skill.execute_skill(execution_params, move_to_above_perturb_pose_params)

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

        terminals = move_down_to_contact_skill.execute_skill(execution_params, move_down_params)

        audio_outcomes = send_audio_outcome_request(audio_outcome_config, terminals[0].stamp)

        print(audio_outcomes['success'])
        if demo:
            if audio_outcomes['success'] == False:
                audio_recovery = send_audio_outcome_request(audio_recovery_config, terminals[0].stamp)
                recovery_action_normalized = audio_recovery['action']
                recovery_action = np.zeros(3)
                recovery_action[0] = (recovery_action_normalized[0] * (config['x_range'][1] - config['x_range'][0])) + config['x_range'][0]
                recovery_action[1] = (recovery_action_normalized[1] * (config['y_range'][1] - config['y_range'][0])) + config['y_range'][0]
                recovery_action[2] = (recovery_action_normalized[2] * (config['theta_range'][1] - config['theta_range'][0])) + config['theta_range'][0] 

                print(f"Ground truth perturb values: {x_perturb}, {y_perturb}, {theta_perturb}")
                print(f"Predicted perturb values: {recovery_action[0]}, {recovery_action[1]}, {recovery_action[2]}")
        
                terminals = move_to_above_correct_pose_skill.execute_skill(execution_params, move_to_above_correct_pose_params)

                expected_result_pub.publish(1)

                terminals = move_down_to_contact_skill.execute_skill(execution_params, move_down_params)

                audio_outcomes = send_audio_outcome_request(audio_outcome_config, terminals[0].stamp)

            if audio_outcomes['success'] == True:
                terminals = push_down_skill.execute_skill(execution_params, config['skill_params']['push_down'])

        else:    
            force_outcomes = send_start_fts_outcome_request(config['fts_detector'])

            terminals = pull_up_skill.execute_skill(execution_params, config['skill_params']['pull_up'])

            force_outcomes = send_end_fts_outcome_request(config['fts_detector'])

            if force_outcomes['success']:
                actual_outcomes.append(0)
            else:
                actual_outcomes.append(1)

            if audio_outcomes['success']:
                predicted_outcomes.append(1)
            else:
                predicted_outcomes.append(0)


            if force_outcomes['success'] != audio_outcomes['success']:
                num_correct_predictions += 1
            num_trials_completed += 1
            logger_string = str(num_correct_predictions) + '/' + str(num_trials_completed)
            logger_pub.publish(logger_string)

            if force_outcomes['success'] == True:

                audio_recovery = send_audio_outcome_request(audio_recovery_config, terminals[0].stamp)
                recovery_action_normalized = audio_recovery['action']
                recovery_action = np.zeros(3)
                recovery_action[0] = (recovery_action_normalized[0] * (config['x_range'][1] - config['x_range'][0])) + config['x_range'][0]
                recovery_action[1] = (recovery_action_normalized[1] * (config['y_range'][1] - config['y_range'][0])) + config['y_range'][0]
                recovery_action[2] = (recovery_action_normalized[2] * (config['theta_range'][1] - config['theta_range'][0])) + config['theta_range'][0] 

                print(f"Ground truth perturb values: {x_perturb}, {y_perturb}, {theta_perturb}")
                print(f"Predicted perturb values: {recovery_action[0]}, {recovery_action[1]}, {recovery_action[2]}")
            
                terminals = move_to_above_correct_pose_skill.execute_skill(execution_params, move_to_above_correct_pose_params)

                expected_result_pub.publish(1)

                terminals = move_down_to_contact_skill.execute_skill(execution_params, move_down_params)

                audio_outcomes = send_audio_outcome_request(audio_outcome_config, terminals[0].stamp)

                print(audio_outcomes['success'])

                force_outcomes = send_start_fts_outcome_request(config['fts_detector'])

                terminals = pull_up_skill.execute_skill(execution_params, config['skill_params']['pull_up'])

                force_outcomes = send_end_fts_outcome_request(config['fts_detector'])

                if force_outcomes['success']:
                    actual_outcomes.append(0)
                else:
                    actual_outcomes.append(1)

                if audio_outcomes['success']:
                    predicted_outcomes.append(1)
                else:
                    predicted_outcomes.append(0)

                if force_outcomes['success'] != audio_outcomes['success']:
                    num_correct_predictions += 1
                num_trials_completed += 1
                logger_string = str(num_correct_predictions) + '/' + str(num_trials_completed)
                logger_pub.publish(logger_string)

            if force_outcomes['success'] == False:
                terminals = push_down_skill.execute_skill(execution_params, config['skill_params']['push_down'])

        terminals = move_to_above_correct_pose_skill.execute_skill(execution_params, move_to_above_correct_pose_params)

        # 3. End rosbag recording
        data_recorder.stop_recording()

        rospy.sleep(1)

        if not demo:
            if force_outcomes['success'] == False:
                labeled_rosbag_path = rosbag_path.split(".bag")[0] + "_success.bag"
                os.rename(rosbag_path, labeled_rosbag_path)
            else:
                labeled_rosbag_path = rosbag_path.split(".bag")[0] + "_failure.bag"
                os.rename(rosbag_path, labeled_rosbag_path)

        expected_result_pub.publish(2)

        if reset:
            place_connector_reset_skill.execute_skill(execution_params)
            reset_connector_skill.execute_skill(execution_params)
        else:
            place_connector_skill.execute_skill(execution_params)
            
    terminals = reset_joints_skill.execute_skill(None)

    cfm = confusion_matrix(actual_outcomes, predicted_outcomes)
    print(cfm)

if __name__ == "__main__":
    run()