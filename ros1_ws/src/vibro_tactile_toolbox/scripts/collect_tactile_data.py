#!/usr/bin/env python3

import rospy
import copy
import numpy as np
import subprocess
import signal
import os

from geometry_msgs.msg import Pose, Twist

from robot_controller.yk_commander import YaskawaRobotCommander

from autolab_core import RigidTransform

STUD_WIDTH = 0.008 # mm
LEGO_BLOCK_HEIGHT=0.0096 #z
# I'm reporting length and width in the nominal/patterened spacing between studs
#  not the measured length and width with clearance added
LEGO_BLOCK_WIDTH=1*STUD_WIDTH #x    #0.0158 (with clearance)
LEGO_BLOCK_LENGTH=2*STUD_WIDTH #y   #0.0318 (with clearance)

results_dir = "/home/mfi/repo/ros1_ws/src/vibro_tactile_toolbox/results"

# def exit_gracefully(signum, frame):
#     # restore the original signal handler as otherwise evil things will happen
#     # in raw_input when CTRL+C is pressed, and our signal handler is not re-entrant
#     signal.signal(signal.SIGINT, original_sigint)

#     try:
#         if raw_input("\nAre you sure you want to quit? (y/n)> ").lower().startswith('y'):
#             sys.exit(1)

#     except KeyboardInterrupt:
#         print("Ok ok, quitting")
#         sys.exit(1)

#     # restore the exit gracefully handler here    
#     signal.signal(signal.SIGINT, exit_gracefully)

def run():
    # Start Node
    rospy.init_node("collect_tactile_data")

    # Messaging Namespace
    namespace = rospy.get_param("collect_tactile_data/namespace")
    root_pwd = rospy.get_param("collect_tactile_data/root_pwd")
    start_trial = rospy.get_param("collect_tactile_data/start_trial")

    # Create Move-It Group for Yaskawa
    robot_mg = YaskawaRobotCommander(namespace)
    
    # Load End-Effector Kinematics
    T_lego_ee = RigidTransform.load(root_pwd+'/config/lego_ee.tf')

    # Load Lego block registration pose 
    T_lego_world = RigidTransform.load(root_pwd+'/config/yk_creator_lego_pose.tf')

    # Make Skills with Perturbations
    # 1. Approach close point
    # 2. Begin rosbag recording
    # 3. Engage brick (with perturbation applied)
    # 4. Execute pick/place rotation
    # 5. Disengage pallet
    # 6. End rosbag recording
    # 7. Goto reset location
    # 8. Hang until user labels trial (success or fail), resets brick, and gives go ahead

    # Set of small perturbations to apply to registered lego location
    # in mm
    perturbations = [
        (0, 0, 0),
        (0.5, 0, 0),
        (-0.5, 0, 0),
        (0, 0.5, 0),
        (0, -0.5, 0),
        (0.5, 0.5, 0),
        (0.5, -0.5, 0),
        (-0.5, 0.5, 0),
        (-0.5, -0.5, 0),
        (0, 0, 0),
    ]

    ## CHECK THESE BEFORE RUNNING###################################################################################################################
    pick_offset_above_location = RigidTransform(translation=np.array([0.0,0.0,-0.05]), 
                                                from_frame='lego', to_frame='lego')
    place_offset_above_location = RigidTransform(translation=np.array([0.0,0.0,-0.05]), 
                                                 from_frame='lego', to_frame='lego')

    T_lego_pick_rotation = RigidTransform(rotation=RigidTransform.y_axis_rotation(np.deg2rad(-15)), 
                                          translation=np.array([LEGO_BLOCK_WIDTH/2,0,LEGO_BLOCK_HEIGHT]), 
                                          from_frame='lego', to_frame='lego')
    T_lego_place_rotation = RigidTransform(rotation=RigidTransform.y_axis_rotation(np.deg2rad(10)), 
                                           translation=np.array([LEGO_BLOCK_WIDTH/2,0,0]), 
                                           from_frame='lego', to_frame='lego')
        
    T_lego_pick_rotation_point_offset = RigidTransform(translation=np.array([LEGO_BLOCK_WIDTH/2,0,LEGO_BLOCK_HEIGHT]), 
                                                       from_frame='lego', to_frame='lego')

    T_lego_place_rotation_point_offset = RigidTransform(translation=np.array([LEGO_BLOCK_WIDTH/2,0,0]), 
                                                        from_frame='lego', to_frame='lego')

    


    skills = [{"skill_type": "pick", "perturbation": np.array(p, dtype=float) / 1000} for p in perturbations]
    skills.extend([{"skill_type": "place", "perturbation": np.array(p, dtype=float) / 1000} for p in perturbations])
    # Example
    # lego_pick_skills[0] : dict({
    #   skill_type: pick/place
    #   perturbation: np.array([0.0001, 0.0001, 0.0000])
    # })

    trials = list(range(len(skills)))
    print(trials)

    print("Returning to Home")
    while not robot_mg.go_home():
        pass

    for i, skill in zip(trials[start_trial:], skills[start_trial:]):
        # get skill data
        skill_type = skill["skill_type"]
        perturbation = skill["perturbation"]

        # current_location established as the registration pose with the perturbation applied
        current_location = T_lego_world
        current_location = current_location * RigidTransform(translation=perturbation, from_frame='lego', to_frame='lego')

        if skill_type == "pick":
            current_offset_above_location = pick_offset_above_location.copy()
            T_lego_rotation = T_lego_pick_rotation.copy()
            T_lego_rotation_point_offset = T_lego_pick_rotation_point_offset.copy()
        elif skill_type == "place":
            current_offset_above_location = place_offset_above_location.copy()
            T_lego_rotation = T_lego_place_rotation.copy()
            T_lego_rotation_point_offset = T_lego_place_rotation_point_offset.copy()
        else:
            raise NotImplementedError

        ### Skill Routine ###

        # 1. Approach close point
        approach_location = current_location * current_offset_above_location
        T_ee_world_target = approach_location * T_lego_ee.inverse()

        goal_pose = T_ee_world_target.pose_msg

        while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True, acc_scaling = 0.5, velocity_scaling=0.5):
            pass

        # 2. Begin rosbag recording
        # side camera
        # wrist camera
        # joint states
        # FTS
        # vibrophone
        rosbag_name = f"vtdata_{skill_type}_trial{i}.bag"
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

        # 3. Engage brick (with perturbation applied)
        T_ee_world_target = current_location * T_lego_ee.inverse()

        goal_pose = T_ee_world_target.pose_msg

        while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
            pass

        # 4. Execute pick/place rotation
        rotation_point = current_location * T_lego_rotation
        
        T_ee_world_target = rotation_point * T_lego_rotation_point_offset.inverse() * T_lego_ee.inverse()

        goal_pose = T_ee_world_target.pose_msg

        while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
            pass

        # 5. Disengage pallet
        T_ee_world_target = approach_location * T_lego_ee.inverse()

        goal_pose = T_ee_world_target.pose_msg

        while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
            pass

        # 6. End rosbag recording
        rosbag_recorder_process.terminate()
        print(f"Ending ROSbag recording.")
        rosbag_t_end = rospy.Time.now()
        duration = rosbag_t_end - rosbag_t_start
        print(f"ROSbag duration: {duration.to_sec():0.2f}")

        # 7. Goto reset location
        print("Returning to Home")
        while not robot_mg.go_home():
            pass

        # 8. Hang until user labels trial (success or fail), resets brick, and gives go ahead
        _ = input(f"Reset lego brick for next trial now\nPress Enter to continue...")
        trial_label = input(f"Label Trial: [Success, Fail]").lower()

        # Rename the saved rosbag with the label
        labeled_rosbag_path = rosbag_path.split(".")[0] + f"_{trial_label}." + rosbag_path.split(".")[1]
        os.rename(rosbag_path, labeled_rosbag_path)

if __name__ == "__main__":
    # original_sigint = signal.getsignal(signal.SIGINT)
    # signal.signal(signal.SIGINT, exit_gracefully)

    run()