#!/usr/bin/env python3

import rospy
import os
import numpy as np

from robot_controller.yk_controller import YaskawaRobotController

from autolab_core import RigidTransform

from skill.lego_skills import PullUp, MoveToAboveLegoPose, MoveToAbovePerturbLegoPose, PickLego, PlaceLego, MoveDown
from skill.util_skills import GoHomeSkill
from outcome.lego_outcome import *

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
    rospy.init_node("collect_tactile_data_new")

    # Messaging Namespace
    namespace = rospy.get_param("collect_tactile_data_new/namespace")
    root_pwd = rospy.get_param("collect_tactile_data_new/root_pwd")
    yaml_file = rospy.get_param("collect_tactile_data_new/config")
    num_trials = rospy.get_param("collect_tactile_data_new/num_trials")
    verbose = rospy.get_param("collect_tactile_data_new/verbose")

    with open(root_pwd+'/config/'+yaml_file) as stream:
        try:
            config = yaml.safe_load(stream)
        except yaml.YAMLError as error:
            print(error)

    results_dir = root_pwd+config['results_dir']+config['block_type']+'/'

    # Instantiate robot controller for Yaskawa API
    robot_commander = YaskawaRobotController(namespace)

    # Load End-Effector Kinematics
    T_lego_ee = RigidTransform.load(root_pwd+config['lego_ee_tf'])

    # Load Lego block registration pose 
    T_lego_world = RigidTransform.load(root_pwd+config['lego_world_tf'])

    ### Skill Routine ###
    params = {'T_lego_ee': T_lego_ee, 
              'verbose': verbose}

    move_to_above_lego_pose_skill = MoveToAboveLegoPose(robot_commander, namespace, params)
    move_to_above_perturb_lego_skill = MoveToAbovePerturbLegoPose(robot_commander, namespace, params)
    pull_up_skill = PullUp(robot_commander, namespace, params)
    move_down_skill = MoveDown(robot_commander, namespace, params)
    place_lego_skill = PlaceLego(robot_commander, namespace, params)
    pick_lego_skill = PickLego(robot_commander, namespace, params)
    home_skill = GoHomeSkill(robot_commander, namespace, params)
    data_recorder = RosbagDataRecorder()

    topics = []

    for topic in config['rosbag_data_recorder']['topics']:
        if 'namespace' in topic:
            topic = topic.replace("namespace", namespace)
        topics.append(topic)

    recording_params = {
        'topics': topics
    }

    # Tasks to do
    for trial_num in range(num_trials):
        x_perturb = np.random.uniform(config['x_range'][0], config['x_range'][1])
        y_perturb = np.random.uniform(config['y_range'][0], config['y_range'][1])
        theta_perturb = np.random.uniform(config['theta_range'][0], config['theta_range'][1])


        # 1. Begin rosbag recording
        rosbag_name = f"trial_{trial_num}-p_{x_perturb:0.4f}_{y_perturb:0.4f}_{theta_perturb:0.4f}.bag"
        rosbag_path = os.path.join(results_dir, rosbag_name)

        data_recorder.start_recording(rosbag_path, recording_params)

        # 2. Determine Skill Parameters
        move_to_above_perturb_lego_params = {
            'T_lego_world': T_lego_world,
            'approach_height_offset': config['approach_height_offset'],
            'place_perturbation': [x_perturb, y_perturb, theta_perturb]
        }

        move_to_above_lego_params = {
            'T_lego_world': T_lego_world,
            'approach_height_offset': config['approach_height_offset'],
        }

        pull_up_params = {
            'lift_height_offset': config['lift_height']
        }

        move_down_to_block_params = {
            'height_offset': config['approach_height_offset']
        }

        move_down_to_reconnect_params = {
            'height_offset': config['lift_height']
        }

        execution_params = {
            'skill_step_delay': 2.0
        }

        place_lego_params = {
            'place_rotation': config['place_rotation'],
            'lift_height_offset': config['approach_height_offset'],
        }

        pick_lego_params = {
            'pick_rotation': config['pick_rotation'],
            'lift_height_offset': config['approach_height_offset'],
        }

        terminals = move_to_above_perturb_lego_skill.execute_skill(execution_params, move_to_above_perturb_lego_params)

        outcomes = send_start_outcome_request({k: config[k] for k in ('fts_detector', 'lego_detector')})

        if outcomes['starting_top'] == 1 and outcomes['starting_bottom'] == 0:
            skill_type = "place"
        elif outcomes['starting_top'] == 0 and outcomes['starting_bottom'] == 1:
            skill_type = "pick"
        else:
            print("Error in skill type. Skipping trial")
            data_recorder.stop_recording()

            labeled_rosbag_path = rosbag_path.split(".")[0] + f"_vision_error." + rosbag_path.split(".")[1]
            os.rename(rosbag_path, labeled_rosbag_path)

            terminals = home_skill.execute_skill(None)
            break

        terminals = move_down_skill.execute_skill(execution_params, move_down_to_block_params)

        terminals = pull_up_skill.execute_skill(execution_params, pull_up_params)

        outcomes = send_end_fts_outcome_request()

        if outcomes['success'] == False:
            terminals = move_to_above_lego_pose_skill.execute_skill(execution_params, move_to_above_lego_params)

            outcomes = send_start_outcome_request()

            if outcomes['starting_top'] == 1 and outcomes['starting_bottom'] == 0:
                skill_type = "place"
            elif outcomes['starting_top'] == 0 and outcomes['starting_bottom'] == 1:
                skill_type = "pick"
            else:
                print("Error in skill type. Skipping trial")
                data_recorder.stop_recording()

                labeled_rosbag_path = rosbag_path.split(".")[0] + f"_vision_error." + rosbag_path.split(".")[1]
                os.rename(rosbag_path, labeled_rosbag_path)

                terminals = home_skill.execute_skill(None)
                break

            terminals = move_down_skill.execute_skill(execution_params, move_down_to_block_params)

            terminals = pull_up_skill.execute_skill(execution_params, pull_up_params)

            outcomes = send_end_fts_outcome_request(config['fts_detector'])

        if outcomes['success'] == False:
            print("Failed to pull up lego. Skipping trial")
            data_recorder.stop_recording()

            labeled_rosbag_path = rosbag_path.split(".")[0] + f"_connection_failure." + rosbag_path.split(".")[1]
            os.rename(rosbag_path, labeled_rosbag_path)

            terminals = home_skill.execute_skill(None)
            continue
        else:
            terminals = move_down_skill.execute_skill(execution_params, move_down_to_reconnect_params)

        if skill_type == "place":
            terminals = place_lego_skill.execute_skill(execution_params, place_lego_params)
        elif skill_type == "pick":
            terminals = pick_lego_skill.execute_skill(execution_params, pick_lego_params)

        outcomes = send_end_vision_outcome_request(config['lego_detector'])

        # 3. End rosbag recording
        data_recorder.stop_recording()

        if outcomes['starting_top'] + outcomes['starting_bottom'] == outcomes['ending_top'] + outcomes['ending_bottom']:
            if outcomes['success']:
                labeled_rosbag_path = rosbag_path.split(".")[0] + "_" + skill_type + "_success." + rosbag_path.split(".")[1]
                os.rename(rosbag_path, labeled_rosbag_path)
            else:
                labeled_rosbag_path = rosbag_path.split(".")[0] + "_" + skill_type + "_failure." + rosbag_path.split(".")[1]
                os.rename(rosbag_path, labeled_rosbag_path)
        else:
            labeled_rosbag_path = rosbag_path.split(".")[0] + "_" + skill_type + "_error." + rosbag_path.split(".")[1]
            os.rename(rosbag_path, labeled_rosbag_path)
            break

        if outcomes['ending_top'] == 1:
            pass
        elif outcomes['ending_bottom'] == 1:
            pass
        
        terminals = home_skill.execute_skill(None)

if __name__ == "__main__":
    run()