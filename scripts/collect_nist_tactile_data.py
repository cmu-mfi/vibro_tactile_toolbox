#!/usr/bin/env python3

import rospy
import os
import numpy as np

from robot_controller.yk_controller import YaskawaRobotController
from gripper_controller.robotiq_hande_controller import RobotiqHandEController

from autolab_core import RigidTransform

from skill.nist_skills import PullUp, MoveToAboveConnectorPose, MoveToAbovePerturbConnectorPose, PickOrPlaceConnector, MoveDown, OpenGripper
from skill.util_skills import GoHomeSkill
from outcome.nist_outcome import *

from data_recorder.rosbag_data_recorder import RosbagDataRecorder

import yaml


def run():
    # Start Node
    rospy.init_node("collect_nist_tactile_data")

    # Messaging Namespace
    namespace = rospy.get_param("collect_nist_tactile_data/namespace")
    root_pwd = rospy.get_param("collect_nist_tactile_data/root_pwd")
    yaml_file = rospy.get_param("collect_nist_tactile_data/config")
    num_trials = rospy.get_param("collect_nist_tactile_data/num_trials")
    start_num = rospy.get_param("collect_nist_tactile_data/start_num")
    connector_type = rospy.get_param("collect_nist_tactile_data/connector_type")
    volume = rospy.get_param("collect_nist_tactile_data/volume")
    velocity_scale = rospy.get_param("collect_nist_tactile_data/velocity_scale")
    verbose = rospy.get_param("collect_nist_tactile_data/verbose")

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

    print(config['fts_detector'])

    data_dir = config['data_dir']+'volume_'+str(volume)+'/'+connector_type+'/vel_'+str(velocity_scale)+'/'

    if not os.path.exists(config['data_dir']+'volume_'+str(volume)):
        os.mkdir(config['data_dir']+'volume_'+str(volume))
    if not os.path.exists(config['data_dir']+'volume_'+str(volume)+'/'+connector_type):
        os.mkdir(config['data_dir']+'volume_'+str(volume)+'/'+connector_type)
    if not os.path.exists(config['data_dir']+'volume_'+str(volume)+'/'+connector_type+'/vel_'+str(velocity_scale)):
        os.mkdir(config['data_dir']+'volume_'+str(volume)+'/'+connector_type+'/vel_'+str(velocity_scale))    

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
    T_hande_ee = RigidTransform.load(root_pwd+config['hande_ee_tf'])

    if connector_type == 'waterproof':
        T_connector_world_pick = RigidTransform.load(root_pwd+config['waterproof_world_pick_tf'])
        T_connector_world_place = RigidTransform.load(root_pwd+config['waterproof_world_place_tf'])
    elif connector_type == 'dsub':
        T_connector_world_pick = RigidTransform.load(root_pwd+config['dsub_world_pick_tf'])
        T_connector_world_place = RigidTransform.load(root_pwd+config['dsub_world_place_tf'])

    ### Skill Routine ###
    params = {'T_hande_ee': T_hande_ee, 
              'verbose': verbose}

    pick_or_place_connector_params = {
        'T_hande_ee': T_hande_ee, 
        'verbose': verbose,
        'T_connector_world': T_connector_world_pick,
        'approach_height_offset': config['approach_height'],
    }

    move_to_above_connector_pose_skill = MoveToAboveConnectorPose(robot_commander, gripper_controller, namespace, params)
    move_to_above_perturb_connector_skill = MoveToAbovePerturbConnectorPose(robot_commander, gripper_controller, namespace, params)
    pull_up_skill = PullUp(robot_commander, gripper_controller, namespace, params)
    move_down_skill = MoveDown(robot_commander, gripper_controller, namespace, params)
    pick_or_place_connector_skill = PickOrPlaceConnector(robot_commander, gripper_controller, namespace, pick_or_place_connector_params)
    home_skill = GoHomeSkill(robot_commander, gripper_controller, namespace, params)
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
        'skill_step_delay': 2.0
    }

    terminals = open_gripper_skill.execute_skill(None)
    terminals = home_skill.execute_skill(None)

    # Tasks to do
    for trial_num in range(start_num, start_num+num_trials):

        pick_execution_params = {'skill_step_delay': 2.0, 'skill_step_params': {'open_or_close_gripper': {'skill':{'pick': True}}}}
        pick_or_place_connector_skill.execute_skill(pick_execution_params, None)

        x_perturb = np.random.uniform(config['x_range'][0], config['x_range'][1])
        y_perturb = np.random.uniform(config['y_range'][0], config['y_range'][1])
        theta_perturb = np.random.uniform(config['theta_range'][0], config['theta_range'][1])

        if move_down_velocity_scaling == -0.1:
            move_down_velocity_scaling = np.random.uniform(0.01, 0.1)

        # 2. Determine Skill Parameters
        move_to_above_perturb_connector_params = {
            'T_connector_world': T_connector_world_place,
            'approach_height_offset': config['approach_height'],
            'place_perturbation': [x_perturb, y_perturb, theta_perturb]
        }

        move_to_above_connector_params = {
            'T_connector_world': T_connector_world_place,
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

        terminals = move_to_above_perturb_connector_skill.execute_skill(execution_params, move_to_above_perturb_connector_params)

        # # 1. Begin rosbag recording
        # rosbag_name = f"trial_{trial_num}-p_{x_perturb:0.4f}_{y_perturb:0.4f}_{theta_perturb:0.4f}_{move_down_velocity_scaling:0.2f}.bag"
        # rosbag_path = os.path.join(data_dir, rosbag_name)

        # data_recorder.start_recording(rosbag_path, recording_params)

        # rospy.sleep(1)

        outcomes = send_start_outcome_request(config['fts_detector'])

        terminals = move_down_skill.execute_skill(execution_params, move_down_params)

        terminals = pull_up_skill.execute_skill(execution_params, pull_up_params)

        outcomes = send_end_fts_outcome_request(config['fts_detector'])

        if outcomes['success'] == False:
            terminals = move_to_above_connector_pose_skill.execute_skill(execution_params, move_to_above_perturb_connector_params)

            outcomes = send_start_outcome_request(config['fts_detector'])

            terminals = move_down_skill.execute_skill(execution_params, move_down_params)

            terminals = pull_up_skill.execute_skill(execution_params, pull_up_params)

            outcomes = send_end_fts_outcome_request(config['fts_detector'])

        terminals = move_to_above_connector_pose_skill.execute_skill(execution_params, move_to_above_perturb_connector_params)

        # 3. End rosbag recording
        #data_recorder.stop_recording()

        rospy.sleep(1)

        # if outcomes['starting_top'] + outcomes['starting_bottom'] == outcomes['ending_top'] + outcomes['ending_bottom']:
        #     if outcomes['success']:
        #         labeled_rosbag_path = rosbag_path.split(".bag")[0] + "_" + skill_type + "_success.bag"
        #         os.rename(rosbag_path, labeled_rosbag_path)
        #     else:
        #         labeled_rosbag_path = rosbag_path.split(".bag")[0] + "_" + skill_type + "_failure.bag"
        #         os.rename(rosbag_path, labeled_rosbag_path)
        # else:
        #     labeled_rosbag_path = rosbag_path.split(".bag")[0] + "_" + skill_type + "_error.bag"
        #     os.rename(rosbag_path, labeled_rosbag_path)
        #     break

        place_execution_params = {'skill_step_delay': 2.0, 'skill_step_params': {'open_or_close_gripper': {'skill':{'pick': False}}}}
        pick_or_place_connector_skill.execute_skill(place_execution_params, None)
        
        terminals = home_skill.execute_skill(None)

if __name__ == "__main__":
    run()