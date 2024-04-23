#!/usr/bin/env python3

import rospy
import os

from robot_controller.yk_controller import YaskawaRobotController

from autolab_core import RigidTransform

from skill.lego_skills import PlaceLegoSkill, PickLegoSkill, PlaceLegoHardcodedCorrectionSkill
from skill.util_skills import GoHomeSkill

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
    perturbations = [(0, 0, -LEGO_BLOCK_HEIGHT/2),
                     (0, STUD_WIDTH/2, 0),
                     (0, -STUD_WIDTH/2, 0),
                     (STUD_WIDTH/2, 0, 0),
                     (-STUD_WIDTH/2, 0, 0),
                     (STUD_WIDTH/2, STUD_WIDTH/2, 0),
                     (-STUD_WIDTH/2, -STUD_WIDTH/2, 0)]

    place_skill = PlaceLegoSkill(robot_commander, namespace)
    place_correction_skill = PlaceLegoHardcodedCorrectionSkill(robot_commander, namespace)
    home_skill = GoHomeSkill(robot_commander, namespace)
    data_recorder = RosbagDataRecorder()
    recording_params = {
        'topics': [f"/side_camera/color/image_cropped",
                f"/camera/color/image_raw",
                f"/{namespace}/joint_states",
                f"/fts",
                f"/audio",
                f"/audio_info",
                f"/{namespace}/terminator/skill_termination_signal",
                f"/outcome/lego_detector",
                f"/outcome/fts_detector"]
    }

    # Tasks to do
    for p in perturbations:
        #p_m = (p[0] / 1000, p[1] / 1000, p[2] / 1000)
        p_m = p
        # 1. Begin rosbag recording
        rosbag_name = f"1x4/place-correct-p_{p_m[0]:0.4f}_{p_m[1]:0.4f}_{p_m[2]:0.4f}"
        rosbag_path = os.path.join(results_dir, rosbag_name)

        data_recorder.start_recording(rosbag_path, recording_params)

        # 2. Begin Skill
        place_lego_params = {
            'T_lego_ee': T_lego_ee,
            'T_lego_world': T_lego_world,
            'approach_height_offset': 0.020,
            'place_rotation': 25.0,
            'place_perturbation': p_m
        }

        execution_params = {
            'skill_step_delay': 2.0
        }

        terminals, outcomes = place_correction_skill.execute_skill(execution_params, place_lego_params)
        #terminals, outcomes = place_skill.execute_skill(execution_params, place_lego_params)

        for i in range(len(terminals)):
            print(f"\n\n=== {place_correction_skill.skill_steps[i]['step_name']} ===" +
                f"\nTerminated with status:\n'{terminals[i].cause}'" +
                f"\nAnd outcome:\n{outcomes[i]}")

        # 3. End rosbag recording
        data_recorder.stop_recording()

        terminals, outcomes = home_skill.execute_skill(None)

        input(f"Reset Lego on arm. Press [Enter] when ready for next trial")


if __name__ == "__main__":
    # original_sigint = signal.getsignal(signal.SIGINT)
    # signal.signal(signal.SIGINT, exit_gracefully)

    run()