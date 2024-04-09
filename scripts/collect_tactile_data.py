#!/usr/bin/env python3

import rospy
import copy
import numpy as np
import subprocess
import signal
import os

from geometry_msgs.msg import Pose, Twist

from robot_controller.yk_controller import YaskawaRobotController

from autolab_core import RigidTransform

from skill.lego_skills import PlaceLegoSkill, PickLegoSkill

STUD_WIDTH = 0.008 # mm
LEGO_BLOCK_HEIGHT=0.0096 #z
# I'm reporting length and width in the nominal/patterened spacing between studs
#  not the measured length and width with clearance added
LEGO_BLOCK_WIDTH=1*STUD_WIDTH #x    #0.0158 (with clearance)
LEGO_BLOCK_LENGTH=2*STUD_WIDTH #y   #0.0318 (with clearance)

results_dir = "/home/mfi/repo/ros1_ws/src/vibro_tactile_toolbox/results"

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
    # side camera
    # wrist camera
    # joint states
    # FTS
    # vibrophone
    rosbag_name = f"name_me.bag"
    rosbag_path = os.path.join(results_dir, rosbag_name)
    command = f"exec rosbag record -O {rosbag_path} " + \
                f"/side_camera/color/image_cropped " + \
                f"/camera/color/image_raw " + \
                f"/{namespace}/joint_states " + \
                f"/fts " + \
                f"/audio " + \
                f"/audio_info "
    print(command)
    print(f"Starting ROSbag recording ...")
    rosbag_t_start = rospy.Time.now()
    rosbag_recorder_process = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True)

    # 2. Begin Skill
    place_lego_params = {
        'T_lego_ee': T_lego_ee,
        'T_lego_world': T_lego_world,
        'place_perturbation_mm': (0, 0, 0)
    }

    place_skill = PlaceLegoSkill(robot_commander, namespace)
    terminals, outcomes = place_skill.execute_skill(None)

    # 3. End rosbag recording
    rosbag_recorder_process.terminate()
    print(f"Ending ROSbag recording.")
    rosbag_t_end = rospy.Time.now()
    duration = rosbag_t_end - rosbag_t_start
    print(f"ROSbag duration: {duration.to_sec():0.2f}")


if __name__ == "__main__":
    # original_sigint = signal.getsignal(signal.SIGINT)
    # signal.signal(signal.SIGINT, exit_gracefully)

    run()