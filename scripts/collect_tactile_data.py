#!/usr/bin/env python3

import rospy
import os

from robot_controller.yk_controller import YaskawaRobotController

from autolab_core import RigidTransform

from skill.lego_skills import PlaceLegoSkill, PickLegoSkill

from data_recorder.rosbag_data_recorder import RosbagDataRecorder


STUD_WIDTH = 0.008 # mm
LEGO_BLOCK_HEIGHT=0.0096 #z
# I'm reporting length and width in the nominal/patterened spacing between studs
#  not the measured length and width with clearance added
LEGO_BLOCK_WIDTH=1*STUD_WIDTH #x    #0.0158 (with clearance)
LEGO_BLOCK_LENGTH=2*STUD_WIDTH #y   #0.0318 (with clearance)

results_dir = "/home/mfi/repos/ros1_ws/src/kevin/vibro_tactile_toolbox/results"

def run():
    # Start Node
    rospy.init_node("collect_tactile_data")

    # Messaging Namespace
    namespace = rospy.get_param("collect_tactile_data/namespace")
    root_pwd = rospy.get_param("collect_tactile_data/root_pwd")
    start_trial = rospy.get_param("collect_tactile_data/start_trial")

    # Instantiate robot controller for Yaskawa API
    robot_commander = YaskawaRobotController(namespace)

    # Load End-Effector Kinematics
    T_lego_ee = RigidTransform.load(root_pwd+'/config/lego_ee.tf')

    # Load Lego block registration pose 
    T_lego_world = RigidTransform.load(root_pwd+'/config/yk_creator_lego_pose.tf')

    ### Skill Routine ###

    # 1. Begin rosbag recording
    rosbag_name = f"name_me"
    rosbag_path = os.path.join(results_dir, rosbag_name)

    data_recorder = RosbagDataRecorder()

    recording_params = {
        'topics': [f"/side_camera/color/image_cropped",
                   f"/camera/color/image_raw",
                   f"/{namespace}/joint_states",
                   f"/fts",
                   f"/audio",
                   f"/audio_info",
                   f"/{namespace}/terminator/skill_termination_signal",
                   f"/outcome/lego_detector"]
    }

    data_recorder.start_recording(rosbag_path, recording_params)

    # 2. Begin Skill
    place_lego_params = {
        'T_lego_ee': T_lego_ee,
        'T_lego_world': T_lego_world,
        'approach_height_offset': 0.025,
        'place_rotation': 25.0,
        'place_perturbation': (0, 0, 0)
    }

    execution_params = {
        'skill_step_delay': 2.0
    }

    place_skill = PlaceLegoSkill(robot_commander, namespace)
    terminals, outcomes = place_skill.execute_skill(execution_params, place_lego_params)

    for i in range(len(terminals)):
        print(f"\n\n=== {place_skill.skill_steps[i]['step_name']} ===" +
              f"\nTerminated with status:\n'{terminals[i].cause}'" +
              f"\nAnd outcome:\n{outcomes[i]}")

    # 3. End rosbag recording
    data_recorder.stop_recording()


if __name__ == "__main__":
    # original_sigint = signal.getsignal(signal.SIGINT)
    # signal.signal(signal.SIGINT, exit_gracefully)

    run()