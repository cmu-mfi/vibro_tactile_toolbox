#!/usr/bin/env python3

import rospy
import os
import numpy as np

from robot_controller.yk_controller import YaskawaRobotController
from gripper_controller.robotiq_hande_controller import RobotiqHandEController

from autolab_core import RigidTransform

from skill.nist_skills import PickConnector, PlaceConnector, PlaceConnectorReset, ResetConnector
from skill.common_skills import ResetJoints, MoveDownToContact, PullUp, OpenGripper, MoveUp, MoveToAboveCorrectPose, MoveToAbovePerturbPose
from outcome.outcome import send_start_fts_outcome_request, send_end_fts_outcome_request
from test.check_ros_topics import check_ros_topics, check_ros_services

from data_recorder.rosbag_data_recorder import RosbagDataRecorder

import yaml


def run():
    # Start Node
    rospy.init_node("collect_nist_audio_data")

    # Messaging Namespace
    namespace = rospy.get_param("collect_nist_audio_data/namespace")
    root_pwd = rospy.get_param("collect_nist_audio_data/root_pwd")
    data_path = rospy.get_param("collect_nist_audio_data/data_dir")
    yaml_file = rospy.get_param("collect_nist_audio_data/config")
    num_trials = rospy.get_param("collect_nist_audio_data/num_trials")
    start_num = rospy.get_param("collect_nist_audio_data/start_num")
    connector_type = rospy.get_param("collect_nist_audio_data/connector_type")
    volume = rospy.get_param("collect_nist_audio_data/volume")
    velocity_scale = rospy.get_param("collect_nist_audio_data/velocity_scale")
    reset = rospy.get_param("collect_nist_audio_data/reset")
    lift = rospy.get_param("collect_nist_audio_data/lift")
    verbose = rospy.get_param("collect_nist_audio_data/verbose")

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

    data_dir = data_path+'/nist/volume_'+str(volume)+'/'+connector_type+'/vel_'+str(velocity_scale)+'/'

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

    # Load End-Effector Kinematics
    T_hande_ee = RigidTransform.load(root_pwd+config['transforms_dir']+config['hande_ee_tf'])

    T_connector_world_pick = RigidTransform.load(root_pwd+config['transforms_dir']+connector_type+'/world_pick.tf')
    T_connector_world_place = RigidTransform.load(root_pwd+config['transforms_dir']+connector_type+'/world_place.tf')
       

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
    pick_connector_skill = PickConnector(robot_commander, gripper_controller, namespace, pick_connector_params)
    place_connector_skill = PlaceConnector(robot_commander, gripper_controller, namespace, place_connector_params)
    
    reset_joints_skill = ResetJoints(robot_commander, gripper_controller, namespace, params)
    open_gripper_skill = OpenGripper(robot_commander, gripper_controller, namespace, params)
    data_recorder = RosbagDataRecorder()

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

    # Tasks to do
    for trial_num in range(start_num, start_num+num_trials):

        pick_connector_skill.execute_skill(execution_params)

        x_perturb = np.random.uniform(config['x_range'][0], config['x_range'][1])
        y_perturb = np.random.uniform(config['y_range'][0], config['y_range'][1])
        theta_perturb = np.random.uniform(config['theta_range'][0], config['theta_range'][1])

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

        terminals = move_to_above_perturb_pose_skill.execute_skill(execution_params, move_to_above_perturb_pose_params)

        all_ros_topics_running = check_ros_topics(topics)
        all_ros_services_running = check_ros_services(config['ros_services'])

        if not (all_ros_topics_running and all_ros_services_running):
            print('Stopping Data Collection Loop.')
            break

        # 1. Begin rosbag recording
        rosbag_name = f"trial_{trial_num}-p_{x_perturb:0.4f}_{y_perturb:0.4f}_{theta_perturb:0.4f}_{move_down_velocity_scaling:0.2f}.bag"
        rosbag_path = os.path.join(data_dir, rosbag_name)

        data_recorder.start_recording(rosbag_path, recording_params)

        rospy.sleep(1)

        terminals = move_down_to_contact_skill.execute_skill(execution_params, move_down_params)

        outcomes = send_start_fts_outcome_request(config['fts_detector'])

        terminals = pull_up_skill.execute_skill(execution_params, config['skill_params']['pull_up'])

        outcomes = send_end_fts_outcome_request(config['fts_detector'])

        if outcomes['success'] == True:
            terminals = move_to_above_correct_pose_skill.execute_skill(execution_params, move_to_above_correct_pose_params)

            terminals = move_down_to_contact_skill.execute_skill(execution_params, move_down_params)

            outcomes = send_start_fts_outcome_request(config['fts_detector'])

            terminals = pull_up_skill.execute_skill(execution_params, config['skill_params']['pull_up'])

            outcomes = send_end_fts_outcome_request(config['fts_detector'])

        terminals = move_to_above_correct_pose_skill.execute_skill(execution_params, move_to_above_correct_pose_params)

        # 3. End rosbag recording
        data_recorder.stop_recording()

        rospy.sleep(1)

        if outcomes['success'] == False:
            labeled_rosbag_path = rosbag_path.split(".bag")[0] + "_success.bag"
            os.rename(rosbag_path, labeled_rosbag_path)
        else:
            labeled_rosbag_path = rosbag_path.split(".bag")[0] + "_failure.bag"
            os.rename(rosbag_path, labeled_rosbag_path)
        
        if reset:
            place_connector_reset_skill.execute_skill(execution_params)
            reset_connector_skill.execute_skill(execution_params)
        else:
            place_connector_skill.execute_skill(execution_params)
            
    terminals = reset_joints_skill.execute_skill(None)

if __name__ == "__main__":
    run()