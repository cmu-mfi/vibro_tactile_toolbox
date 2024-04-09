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

from vibro_tactile_toolbox.msg import TerminationSignal
from vibro_tactile_toolbox.srv import *
from vibro_tactile_toolbox.msg import *

from typing import Tuple, List

### Lego specific globals ###

STUD_WIDTH = 0.008 # 8 mm
LEGO_BLOCK_HEIGHT=0.0096 # z 9.6mm
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

    def __init__(self, robot_commander: BaseRobotCommander, namespace: str):

        super().__init__(robot_commander, namespace)

        self.skill_steps = [
            {'step_name': 'go_to_approach_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.approach_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.approach_pose_msg),
             'outcome': lambda param: self.send_start_outcome_request(param)},
            {'step_name': 'engage_brick',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.place_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(engage_termination_config, self.place_pose_msg)},
            {'step_name': 'pull_up',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.servo_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(servo_termination_config, self.servo_pose_msg),
             'outcome': lambda param: self.send_end_fts_outcome_request(param)},
            {'step_name': 'reengage_brick',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.place_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(engage_termination_config, self.place_pose_msg)},
            {'step_name': 'release_brick',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.release_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(release_termination_config, self.release_pose_msg),
             'outcome': lambda param: self.send_end_vision_outcome_request(param)},
            {'step_name': 'disengage_brick',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.approach_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.approach_pose_msg)}
        ]

    def execute_skill(self, params, place_lego_params) -> Tuple[List[TerminationSignal], List[int]]:
        # QOL 
        if 'T_lego_ee' not in place_lego_params:
            print(f"PlaceLegoSkill expects end effector transform: place_lego_params['T_lego_ee'] = RigidTransform()")
        if 'T_lego_world' not in place_lego_params:
            print(f"PlaceLegoSkill expects target brick pose transform: place_lego_params['T_lego_world'] = RigidTransform()")
        if 'approach_height_offset' not in place_lego_params:
            print(f"PlaceLegoSkill expects an approach height offset (meters): place_lego_params['approach_height_offset'] = float(0.010 m)")
        if 'place_rotation' not in place_lego_params:
            print(f"PlaceLegoSkill expects a release rotation (deg): place_lego_params['place_rotation'] = float(30 deg)")


        self.T_lego_ee = place_lego_params['T_lego_ee']
        self.T_lego_world = place_lego_params['T_lego_world']
        self.approach_height_offset = place_lego_params['approach_height_offset']
        self.place_rotation = place_lego_params['place_rotation']

        self.T_lego_approach = RigidTransform(
            translation=np.array([0.0, 0.0, -abs(self.approach_height_offset)]),
            from_frame='lego', to_frame='lego'
        )
        self.T_lego_rotation = RigidTransform(
            rotation=RigidTransform.y_axis_rotation(np.deg2rad(self.place_rotation)), 
            translation=np.array([LEGO_BLOCK_WIDTH/2,0,0]), 
            from_frame='lego', to_frame='lego'
        )
        self.T_lego_rotation_point_offset = RigidTransform(
            translation=np.array([LEGO_BLOCK_WIDTH/2,0,0]), 
            from_frame='lego', to_frame='lego'
        )
        self.T_lego_servo = RigidTransform(
            translation=np.array([0.0, 0.0, -0.001]),
            from_frame='lego', to_frame='lego'
        )


        self.T_place_target = self.T_lego_world.copy()
        if place_lego_params['place_perturbation']:
            self.place_perturbation = np.array(place_lego_params['place_perturbation'])
            self.T_place_target *= RigidTransform(
                translation=self.place_perturbation, 
                from_frame='lego', to_frame='lego'
            )
        
        self.place_pose = self.T_place_target * self.T_lego_ee.inverse()
        self.place_pose_msg = self.place_pose.pose_msg

        self.approach_pose = self.T_place_target * self.T_lego_approach * self.T_lego_ee.inverse()
        self.approach_pose_msg = self.approach_pose.pose_msg

        self.release_pose = self.T_place_target * self.T_lego_rotation * self.T_lego_rotation_point_offset.inverse() * self.T_lego_ee.inverse()
        self.release_pose_msg = self.release_pose.pose_msg

        self.servo_pose = self.T_place_target * self.T_lego_servo * self.T_lego_ee.inverse()
        self.servo_pose_msg = self.servo_pose.pose_msg
        return super().execute_skill(params)

    def send_start_outcome_request(self, param):
        return -1
        raise NotImplementedError
    
    def send_end_fts_outcome_request(self, param):
        return -1
        raise NotImplementedError
    
    def send_end_vision_outcome_request(self, param):
        return -1
        raise NotImplementedError
    
    def calculate_approach_pose(self, param):
        place_perturbation = np.array(param['place_perturbation_mm']) / 1000.0
        T_place_offset = RigidTransform(
            translation=place_perturbation, 
            from_frame='lego', to_frame='lego'
        )
        T_place_target = self.T_lego_world * T_place_offset 
        approach_pose = T_place_target * self.T_lego_approach * self.T_lego_ee.inverse()
        approach_pose = approach_pose.pose_msg
        self.robot_commander.go_to_pose_goal(approach_pose, wait=False),

class PickLegoSkill(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, params):

        super().__init__(robot_commander)

        self.T_lego_ee = params['T_lego_ee']
        self.T_lego_world = params['T_lego_world']

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