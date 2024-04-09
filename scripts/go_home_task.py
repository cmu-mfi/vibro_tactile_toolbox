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

from skill.util_skills import GoHomeSkill

STUD_WIDTH = 0.008 # mm
LEGO_BLOCK_HEIGHT=0.0096 #z
# I'm reporting length and width in the nominal/patterened spacing between studs
#  not the measured length and width with clearance added
LEGO_BLOCK_WIDTH=1*STUD_WIDTH #x    #0.0158 (with clearance)
LEGO_BLOCK_LENGTH=2*STUD_WIDTH #y   #0.0318 (with clearance)

results_dir = "/home/mfi/repo/ros1_ws/src/vibro_tactile_toolbox/results"

def run():
    # Start Node
    rospy.init_node("go_home_task")

    # Messaging Namespace
    namespace = rospy.get_param("go_home_task/namespace")
    root_pwd = rospy.get_param("go_home_task/root_pwd")
    start_trial = rospy.get_param("go_home_task/start_trial")

    # Instantiate robot controller for Yaskawa API
    robot_commander = YaskawaRobotController(namespace)

    ### Skill Routine ###

    # 1. Begin rosbag recording
    # side camera
    # wrist camera
    # joint states
    # FTS
    # vibrophone
    # rosbag_name = f"name_me.bag"
    # rosbag_path = os.path.join(results_dir, rosbag_name)
    # command = f"exec rosbag record -O {rosbag_path} " + \
    #             f"/side_camera/color/image_cropped " + \
    #             f"/camera/color/image_raw " + \
    #             f"/{namespace}/joint_states " + \
    #             f"/fts " + \
    #             f"/audio " + \
    #             f"/audio_info "
    # print(command)
    # print(f"Starting ROSbag recording ...")
    # rosbag_t_start = rospy.Time.now()
    # rosbag_recorder_process = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True)

    # 2. Begin Skill
    home_skill = GoHomeSkill(robot_commander, namespace)
    terminals, outcomes = home_skill.execute_skill(None)
    
    for i, terminal in enumerate(terminals):
        print(f"step {i} termination cause:\n{terminal.cause}\n")

    # 3. End rosbag recording
    # rosbag_recorder_process.terminate()
    # print(f"Ending ROSbag recording.")
    # rosbag_t_end = rospy.Time.now()
    # duration = rosbag_t_end - rosbag_t_start
    # print(f"ROSbag duration: {duration.to_sec():0.2f}")


if __name__ == "__main__":
    # original_sigint = signal.getsignal(signal.SIGINT)
    # signal.signal(signal.SIGINT, exit_gracefully)

    run()