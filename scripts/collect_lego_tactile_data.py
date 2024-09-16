#!/usr/bin/env python3

import rospy
import os
import numpy as np

from robot_controller.yk_controller import YaskawaRobotController
from gripper_controller.lego_gripper_controller import LegoGripperController

from autolab_core import RigidTransform

from skill.lego_skills import PullUp, MoveToAboveLegoPose, MoveToAbovePerturbLegoPose, PickLego, PlaceLego, MoveDown
from skill.util_skills import GoHomeSkill
from outcome.outcome import *

from data_recorder.rosbag_data_recorder import RosbagDataRecorder

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
    rospy.init_node("collect_lego_tactile_data")

    # Messaging Namespace
    namespace = rospy.get_param("collect_lego_tactile_data/namespace")
    root_pwd = rospy.get_param("collect_lego_tactile_data/root_pwd")
    yaml_file = rospy.get_param("collect_lego_tactile_data/config")
    num_trials = rospy.get_param("collect_lego_tactile_data/num_trials")
    start_num = rospy.get_param("collect_lego_tactile_data/start_num")
    block_type = rospy.get_param("collect_lego_tactile_data/block_type")
    volume = rospy.get_param("collect_lego_tactile_data/volume")
    velocity_scale = rospy.get_param("collect_lego_tactile_data/velocity_scale")
    verbose = rospy.get_param("collect_lego_tactile_data/verbose")

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

    data_dir = config['data_dir']+'volume_'+str(volume)+'/'+block_type+'/vel_'+str(velocity_scale)+'/'

    if not os.path.exists(config['data_dir']+'volume_'+str(volume)):
        os.mkdir(config['data_dir']+'volume_'+str(volume))
    if not os.path.exists(config['data_dir']+'volume_'+str(volume)+'/'+block_type):
        os.mkdir(config['data_dir']+'volume_'+str(volume)+'/'+block_type)
    if not os.path.exists(config['data_dir']+'volume_'+str(volume)+'/'+block_type+'/vel_'+str(velocity_scale)):
        os.mkdir(config['data_dir']+'volume_'+str(volume)+'/'+block_type+'/vel_'+str(velocity_scale))    

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
    gripper_controller = LegoGripperController(namespace)

    # Load End-Effector Kinematics
    T_lego_ee = RigidTransform.load(root_pwd+config['lego_ee_tf'])

    # Load Lego block registration pose 
    T_lego_world = {}

    for tf in os.listdir(root_pwd+config['lego_world_tf']):
        x_loc = tf.split('_')[2]
        y_loc = tf.split('_')[3][:-3]
        T_lego_world[x_loc+','+y_loc] = RigidTransform.load(root_pwd+config['lego_world_tf']+tf)

    block_x_loc = np.random.randint(config['block_x_range'][0], config['block_x_range'][1])
    block_y_loc = np.random.randint(config['block_y_range'][0], config['block_y_range'][1])

    tmp_T_lego_world = determine_next_pose(T_lego_world, block_x_loc, block_y_loc)

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

    # Tasks to do
    for trial_num in range(start_num, start_num+num_trials):
        # Load temporary lego block registration pose 
        tmp_T_lego_world = RigidTransform.load(root_pwd+config['tmp_lego_world_tf'])

        x_perturb = np.random.uniform(config['x_range'][0], config['x_range'][1])
        y_perturb = np.random.uniform(config['y_range'][0], config['y_range'][1])
        theta_perturb = np.random.uniform(config['theta_range'][0], config['theta_range'][1])

        if move_down_velocity_scaling == -0.1:
            move_down_velocity_scaling = np.random.uniform(0.01, 0.1)

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

        if outcomes['starting_top'] == 1 and outcomes['starting_bottom'] == 0:
            skill_type = "place"
        elif outcomes['starting_top'] == 0 and outcomes['starting_bottom'] == 1:
            skill_type = "pick"
        else:
            print("Error in skill type. Skipping trial")
            data_recorder.stop_recording()

            rospy.sleep(1)

            labeled_rosbag_path = rosbag_path.split(".bag")[0] + f"_vision_error.bag"
            os.rename(rosbag_path, labeled_rosbag_path)

            terminals = home_skill.execute_skill(None)
            break

        terminals = move_down_skill.execute_skill(execution_params, move_down_params)

        terminals = pull_up_skill.execute_skill(execution_params, pull_up_params)

        outcomes = send_end_fts_outcome_request(config['fts_detector'])

        if outcomes['success'] == False:
            terminals = move_to_above_lego_pose_skill.execute_skill(execution_params, move_to_above_lego_params)

            start_fts_outcome = send_start_fts_outcome_request(config['fts_detector'])
            outcomes = send_start_vision_outcome_request(config['lego_detector'])

            if outcomes['starting_top'] == 1 and outcomes['starting_bottom'] == 0:
                skill_type = "place"
            elif outcomes['starting_top'] == 0 and outcomes['starting_bottom'] == 1:
                skill_type = "pick"
            else:
                print("Error in skill type. Skipping trial")
                data_recorder.stop_recording()

                rospy.sleep(1)

                labeled_rosbag_path = rosbag_path.split(".bag")[0] + f"_vision_error.bag"
                os.rename(rosbag_path, labeled_rosbag_path)

                terminals = home_skill.execute_skill(None)
                break

            terminals = move_down_skill.execute_skill(execution_params, move_down_params)

            terminals = pull_up_skill.execute_skill(execution_params, pull_up_params)

            outcomes = send_end_fts_outcome_request(config['fts_detector'])

        if outcomes['success'] == False:
            print("Failed to pull up lego. Skipping trial")
            data_recorder.stop_recording()

            rospy.sleep(1)

            labeled_rosbag_path = rosbag_path.split(".bag")[0] + f"_connection_failure.bag"
            os.rename(rosbag_path, labeled_rosbag_path)

            if skill_type == 'place':
                block_x_loc = np.random.randint(config['block_x_range'][0], config['block_x_range'][1])
                block_y_loc = np.random.randint(config['block_y_range'][0], config['block_y_range'][1])

                tmp_T_lego_world = determine_next_pose(T_lego_world, block_x_loc, block_y_loc)

                tmp_T_lego_world.save(root_pwd+config['tmp_lego_world_tf'])

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

        if outcomes['ending_top'] == 1:
            pass
        elif outcomes['ending_bottom'] == 1:
            pass
        
        terminals = home_skill.execute_skill(None)

        if skill_type == "pick" and outcomes['success']:
            block_x_loc = np.random.randint(config['block_x_range'][0], config['block_x_range'][1])
            block_y_loc = np.random.randint(config['block_y_range'][0], config['block_y_range'][1])

            tmp_T_lego_world = determine_next_pose(T_lego_world, block_x_loc, block_y_loc)

            tmp_T_lego_world.save(root_pwd+config['tmp_lego_world_tf'])

if __name__ == "__main__":
    run()