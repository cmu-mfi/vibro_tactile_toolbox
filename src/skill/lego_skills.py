#!/usr/bin/env python3

import copy
import numpy as np
import subprocess
import signal
import os
import rospy
from geometry_msgs.msg import Pose
from autolab_core import RigidTransform

from robot_controller.base_robot_commander import BaseRobotCommander
from gripper_controller.base_gripper_controller import BaseGripperController
from skill.base_skill import BaseSkill
import terminator.utils as t_utils

from vibro_tactile_toolbox.msg import TerminationSignal

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
# Terminate if Fz exceeds force to join legos with good alignment
# 10N is an estimate
engage_termination_config = {
    'time': {'duration': 10.0},
    'pose': {'pos_tolerance': 0.0001,
                'orient_tolerance': 0.01,
                'pose': None},
    'fts': {'check_rate_ns': 1E6,
            'threshold': {
                'force': {
                    'x': [-10, 10],
                    'y': [-10, 10],
                    'z': [0, 10]},
                'torque': {
                    'x': [-1, 1],
                    'y': [-1, 1],
                    'z': [-1, 1]}
            }}
}
# Releasing a brick through rotation
# Try not to terminate from fts unless the readings may cause an E-stop
release_termination_config = {
    'time': {'duration': 10.0},
    'pose': {'pos_tolerance': 0.001,
                'orient_tolerance': 0.01,
                'pose': None},
    'fts': {'check_rate_ns': 1E6,
            'threshold': {
                'force': {
                    'x': [-50, 50],
                    'y': [-50, 50],
                    'z': [-100, 100]},
                'torque': {
                    'x': [-2, 2],
                    'y': [-2, 1],
                    'z': [-2, 2]}
            }}
}

def add_termination_pose(termination_config, pose : Pose):
    t_cfg = copy.deepcopy(termination_config)
    t_cfg['pose']['pose'] = t_utils.pose_to_dict(pose)
    return t_cfg

class PlaceLego(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, gripper_controller: BaseGripperController, namespace: str, params=None):

        super().__init__(robot_commander, gripper_controller, namespace, params)

        if 'T_lego_ee' not in self.params:
            print(f"PlaceLego expects end effector transform: params['T_lego_ee'] = RigidTransform()")

        self.T_lego_ee = self.params['T_lego_ee']

        self.skill_steps = [
            {'step_name': 'rotate_end_effector',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.rotate_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(release_termination_config, self.rotate_pose_msg)},
            {'step_name': 'lift_up',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.lift_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.lift_pose_msg)}
        ]

    def execute_skill(self, execution_params, skill_params) -> Tuple[List[TerminationSignal], List[int]]:
        # QOL 
        if 'place_rotation' not in skill_params:
            print(f"PlaceLego expects a release rotation (deg): skill_params['place_rotation'] = float(30 deg)")
        if 'lift_height_offset' not in skill_params:
            print(f"PlaceLego expects an lift height offset (meters): skill_params['lift_height_offset'] = float(0.020 m)")

        self.place_rotation = skill_params['place_rotation']
        self.lift_height_offset = skill_params['lift_height_offset']

        
        self.T_lego_rotation = RigidTransform(
            rotation=RigidTransform.y_axis_rotation(np.deg2rad(self.place_rotation)), 
            translation=np.array([LEGO_BLOCK_WIDTH/2,0,0]), 
            from_frame='lego', to_frame='lego'
        )

        self.T_ee_lift = RigidTransform(
            translation=np.array([0.0, 0.0, -abs(self.lift_height_offset)]),
            from_frame='ee', to_frame='ee'
        )

        self.T_lego_rotation_point_offset = RigidTransform(
            translation=np.array([LEGO_BLOCK_WIDTH/2,0,0]), 
            from_frame='lego', to_frame='lego'
        )

        current_pose_msg = self.robot_commander.get_current_pose()
        current_pose = RigidTransform.from_pose_msg(current_pose_msg, from_frame='ee')

        self.lift_pose = current_pose * self.T_ee_lift
        self.lift_pose_msg = self.lift_pose.pose_msg

        self.rotate_pose = current_pose * self.T_lego_ee * self.T_lego_rotation * self.T_lego_rotation_point_offset.inverse() * self.T_lego_ee.inverse()
        self.rotate_pose_msg = self.rotate_pose.pose_msg

        return super().execute_skill(execution_params)
    
class PickLego(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, gripper_controller: BaseGripperController, namespace: str, params=None):

        super().__init__(robot_commander, gripper_controller, namespace, params)

        if 'T_lego_ee' not in self.params:
            print(f"PickLego expects end effector transform: params['T_lego_ee'] = RigidTransform()")

        self.T_lego_ee = self.params['T_lego_ee']

        self.skill_steps = [
            {'step_name': 'rotate_end_effector',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.rotate_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(release_termination_config, self.rotate_pose_msg)},
            {'step_name': 'lift_up',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.lift_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.lift_pose_msg)}
        ]

    def execute_skill(self, execution_params, skill_params) -> Tuple[List[TerminationSignal], List[int]]:
        # QOL 
        if 'pick_rotation' not in skill_params:
            print(f"PickLego expects a release rotation (deg): skill_params['pick_rotation'] = float(30 deg)")
        if 'lift_height_offset' not in skill_params:
            print(f"PickLego expects an lift height offset (meters): skill_params['lift_height_offset'] = float(0.020 m)")

        self.pick_rotation = skill_params['pick_rotation']
        self.lift_height_offset = skill_params['lift_height_offset']

        
        self.T_lego_rotation = RigidTransform(
            rotation=RigidTransform.y_axis_rotation(np.deg2rad(-self.pick_rotation)), 
            translation=np.array([LEGO_BLOCK_WIDTH/2,0,LEGO_BLOCK_HEIGHT]), 
            from_frame='lego', to_frame='lego'
        )

        self.T_ee_lift = RigidTransform(
            translation=np.array([0.0, 0.0, -abs(self.lift_height_offset)]),
            from_frame='ee', to_frame='ee'
        )

        self.T_lego_rotation_point_offset = RigidTransform(
            translation=np.array([LEGO_BLOCK_WIDTH/2,0,LEGO_BLOCK_HEIGHT]), 
            from_frame='lego', to_frame='lego'
        )

        current_pose_msg = self.robot_commander.get_current_pose()
        current_pose = RigidTransform.from_pose_msg(current_pose_msg, from_frame='ee')

        self.lift_pose = current_pose * self.T_ee_lift
        self.lift_pose_msg = self.lift_pose.pose_msg

        self.rotate_pose = current_pose * self.T_lego_ee * self.T_lego_rotation * self.T_lego_rotation_point_offset.inverse() * self.T_lego_ee.inverse()
        self.rotate_pose_msg = self.rotate_pose.pose_msg

        return super().execute_skill(execution_params)