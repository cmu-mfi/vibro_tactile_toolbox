#!/usr/bin/env python3

import copy
import numpy as np
import subprocess
import signal
import os

from geometry_msgs.msg import Pose, Twist
from autolab_core import RigidTransform

from robot_controller.base_robot_commander import BaseRobotCommander
from skill.base_skill import BaseSkill
import terminator.utils as t_utils

from typing import Tuple, List

### Lego specific globals ###

STUD_WIDTH = 0.008 # mm
LEGO_BLOCK_HEIGHT=0.0096 #z
LEGO_BLOCK_WIDTH=1*STUD_WIDTH #x    #0.0158 (with clearance)
LEGO_BLOCK_LENGTH=2*STUD_WIDTH #y   #0.0318 (with clearance)

### Termination Configs for Lego Skills ###
# Rapid motion
rapid_termination_config = {
    'time': {'duration': 10.0},
    'pose': {'pos_tolerance': 0.001,
                'orient_tolerance': 0.01,
                'pose': None},
}
# Engaging a brick on the build plate
engage_termination_config = {
    'time': {'duration': 10.0},
    'pose': {'pos_tolerance': 0.001,
                'orient_tolerance': 0.01,
                'pose': None},
    'fts': {'check_rate_ns': 1E6,
            'threshold': {
                'force': {
                    'x': 10,
                    'y': 10,
                    'z': 20},
                'torque': {
                    'x': 1.0,
                    'y': 1.0,
                    'z': 1.0}
            }}
}
# Releasing a brick through rotation
release_termination_config = {
    'time': {'duration': 10.0},
    'pose': {'pos_tolerance': 0.001,
                'orient_tolerance': 0.01,
                'pose': None},
    'fts': {'check_rate_ns': 1E6,
            'threshold': {
                'force': {
                    'x': 10,
                    'y': 10,
                    'z': 20},
                'torque': {
                    'x': 1.0,
                    'y': 1.0,
                    'z': 1.0}
            }}
}
# Pulling up on a brick to check if connected
servo_termination_config = {
    'time': {'duration': 10.0},
    'pose': {'pos_tolerance': 0.001,
                'orient_tolerance': 0.01,
                'pose': None},
    'fts': {'check_rate_ns': 1E6,
            'threshold': {
                'force': {
                    'x': 10,
                    'y': 10,
                    'z': 20},
                'torque': {
                    'x': 1.0,
                    'y': 1.0,
                    'z': 1.0}
            }}
}

def add_termination_pose(termination_config, pose : Pose):
    t_cfg = copy.deepcopy(termination_config)
    t_cfg['pose']['pose'] = t_utils.pose_to_dict(pose)
    return t_cfg

class PlaceLegoSkill(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, params):

        super().__init__(robot_commander)
        
        T_lego_ee = params['T_lego_ee']
        T_lego_world = params['T_lego_world']

        T_lego_approach = RigidTransform(
            translation=np.array([0.0, 0.0, -0.050])
        )
        T_lego_rotation = RigidTransform(
            rotation=RigidTransform.y_axis_rotation(np.deg2rad(10)), 
            translation=np.array([LEGO_BLOCK_WIDTH/2,0,0]), 
            from_frame='lego', to_frame='lego'
        )
        T_lego_rotation_point_offset = RigidTransform(
            translation=np.array([LEGO_BLOCK_WIDTH/2,0,0]), 
            from_frame='lego', to_frame='lego'
        )
        T_lego_servo = RigidTransform(
            translation=np.array([0.0, 0.0, -0.001]),
            from_frame='lego', to_frame='lego'
        )


        T_place_target = T_lego_world
        if params['place_perturbation_mm']:
            place_perturbation = np.array(params['place_perturbation_mm']) / 1000.0
            # Should this be a perturbation in the world frame or EE frame?
            T_place_target *= RigidTransform(
                translation=place_perturbation, 
                from_frame='lego', to_frame='lego'
            )
        
        place_pose = T_place_target * T_lego_ee.inverse()
        place_pose = place_pose.pose_msg

        approach_pose = T_place_target * T_lego_approach * T_lego_ee.inverse()
        approach_pose = approach_pose.pose_msg

        release_pose = T_place_target * T_lego_rotation 
        release_pose *= T_lego_rotation_point_offset.inverse() * T_lego_ee.inverse()
        release_pose = release_pose.pose_msg

        servo_pose = T_place_target * T_lego_servo * T_lego_ee.inverse()
        servo_pose = servo_pose.pose_msg


        self.skill_steps = [
            {'step_name': 'go_to_approach_pose',
             'command': lambda: self.robot_commander.go_to_pose_goal(approach_pose, wait=False),
             'termination_cfg': add_termination_pose(rapid_termination_config, approach_pose)},
            {'step_name': 'engage_brick',
             'command': lambda: self.robot_commander.go_to_pose_goal(place_pose, wait=False),
             'termination_cfg': add_termination_pose(engage_termination_config, place_pose)},
            {'step_name': 'pull_up',
             'command': lambda: self.robot_commander.go_to_pose_goal(servo_pose, wait=False),
             'termination_cfg': add_termination_pose(servo_termination_config, servo_pose)},
            {'step_name': 'reengage_brick',
             'command': lambda: self.robot_commander.go_to_pose_goal(place_pose, wait=False),
             'termination_cfg': add_termination_pose(engage_termination_config, place_pose)},
            {'step_name': 'release_brick',
             'command': lambda: self.robot_commander.go_to_pose_goal(release_pose, wait=False),
             'termination_cfg': add_termination_pose(release_termination_config, release_pose)},
            {'step_name': 'disengage_brick',
             'command': lambda: self.robot_commander.go_to_pose_goal(approach_pose, wait=False),
             'termination_cfg': add_termination_pose(rapid_termination_config, approach_pose)}
        ]

class PickLegoSkill(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, params):

        super().__init__(robot_commander)

        T_lego_ee = params['T_lego_ee']
        T_lego_world = params['T_lego_world']

        T_lego_approach = RigidTransform(
            translation=np.array([0.0, 0.0, -0.050])
        )
        T_lego_rotation = RigidTransform(
            rotation=RigidTransform.y_axis_rotation(np.deg2rad(-15)), 
            translation=np.array([LEGO_BLOCK_WIDTH/2,0,LEGO_BLOCK_HEIGHT]), 
            from_frame='lego', to_frame='lego'
        )
        T_lego_rotation_point_offset = RigidTransform(
            translation=np.array([LEGO_BLOCK_WIDTH/2,0,LEGO_BLOCK_HEIGHT]), 
            from_frame='lego', to_frame='lego'
        )
        T_lego_servo = RigidTransform(
            translation=np.array([0.0, 0.0, -0.001]),
            from_frame='lego', to_frame='lego'
        )


        T_place_target = T_lego_world
        if params['place_perturbation_mm']:
            place_perturbation = np.array(params['place_perturbation_mm']) / 1000.0
            # Should this be a perturbation in the world frame or EE frame?
            T_place_target *= RigidTransform(
                translation=place_perturbation, 
                from_frame='lego', to_frame='lego'
            )
        
        place_pose = T_place_target * T_lego_ee.inverse()
        place_pose = place_pose.pose_msg

        approach_pose = T_place_target * T_lego_approach * T_lego_ee.inverse()
        approach_pose = approach_pose.pose_msg

        release_pose = T_place_target * T_lego_rotation 
        release_pose *= T_lego_rotation_point_offset.inverse() * T_lego_ee.inverse()
        release_pose = release_pose.pose_msg

        servo_pose = T_place_target * T_lego_servo * T_lego_ee.inverse()
        servo_pose = servo_pose.pose_msg


        self.skill_steps = [
            {'step_name': 'go_to_approach_pose',
             'command': lambda: self.robot_commander.go_to_pose_goal(approach_pose, wait=False),
             'termination_cfg': add_termination_pose(rapid_termination_config, approach_pose)},
            {'step_name': 'engage_brick',
             'command': lambda: self.robot_commander.go_to_pose_goal(place_pose, wait=False),
             'termination_cfg': add_termination_pose(engage_termination_config, place_pose)},
            {'step_name': 'pull_up',
             'command': lambda: self.robot_commander.go_to_pose_goal(servo_pose, wait=False),
             'termination_cfg': add_termination_pose(servo_termination_config, servo_pose)},
            {'step_name': 'reengage_brick',
             'command': lambda: self.robot_commander.go_to_pose_goal(place_pose, wait=False),
             'termination_cfg': add_termination_pose(engage_termination_config, place_pose)},
            {'step_name': 'release_brick',
             'command': lambda: self.robot_commander.go_to_pose_goal(release_pose, wait=False),
             'termination_cfg': add_termination_pose(release_termination_config, release_pose)},
            {'step_name': 'disengage_brick',
             'command': lambda: self.robot_commander.go_to_pose_goal(approach_pose, wait=False),
             'termination_cfg': add_termination_pose(rapid_termination_config, approach_pose)}
        ]

class ResetLegoPalletSkill(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander):

        super().__init__(robot_commander)

        self.skill_steps = [
             {'step_name': 'go_to_home_joints',
              'command': lambda: self.robot_commander.go_to_joints([0, 0, 0, 0, -np.pi/2, 0], wait=False),
              'termination_cfg': {'joint': {'positions': [0, 0, 0, 0, -np.pi/2, 0]}}}
        ]

class ResetLegoGripperSkill(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander):

        super().__init__(robot_commander)

        self.skill_steps = [
             {'step_name': 'go_to_home_joints',
              'command': lambda: self.robot_commander.go_to_joints([0, 0, 0, 0, -np.pi/2, 0], wait=False),
              'termination_cfg': {'joint': {'positions': [0, 0, 0, 0, -np.pi/2, 0]}}}
        ]