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
                    'z': [-3, 10]},
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
# Pulling up on a brick to check if connected
# Terminate if z goes positive
# Terminate if any substantial reaction otherwise
servo_termination_config = {
    'time': {'duration': 10.0},
    'pose': {'pos_tolerance': 0.001,
                'orient_tolerance': 0.01,
                'pose': None},
    'fts': {'check_rate_ns': 1E6,
            'threshold': {
                'force': {
                    'x': [-10, 10],               # any substantial reaction force in x-y
                    'y': [-10, 10],
                    'z': [-float('inf'), 2]},   # reaction force pulling away in z
                'torque': {
                    'x': [-1, 1],               # any substantial reaction torque
                    'y': [-1, 1],
                    'z': [-1, 1]}
            }}
}

def add_termination_pose(termination_config, pose : Pose):
    t_cfg = copy.deepcopy(termination_config)
    t_cfg['pose']['pose'] = t_utils.pose_to_dict(pose)
    return t_cfg

class PlaceLegoHardcodedCorrectionSkill(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, namespace: str, params=None):

        super().__init__(robot_commander, namespace, params)

        if 'T_lego_ee' not in self.params:
            print(f"PlaceLegoHardcodedCorrectionSkill expects end effector transform: params['T_lego_ee'] = RigidTransform()")

        self.T_lego_ee = self.params['T_lego_ee']

        self.skill_steps = [
            {'step_name': 'go_to_approach_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.approach_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.approach_pose_msg)},
            {'step_name': 'engage_brick_bad',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.place_pose_bad_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(engage_termination_config, self.place_pose_bad_msg)},
            {'step_name': 'pull_up',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.servo_pose_bad_msg, wait=False, velocity_scaling=0.001),
             'termination_cfg': lambda param: add_termination_pose(servo_termination_config, self.servo_pose_bad_msg),
             'outcome': lambda param: send_end_fts_outcome_request(param)},
            {'step_name': 'reset_to_approach_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.approach_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.approach_pose_msg)},
            {'step_name': 'engage_brick_good',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.place_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(engage_termination_config, self.place_pose_msg)},
            {'step_name': 'pull_up',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.servo_pose_msg, wait=False, velocity_scaling=0.001),
             'termination_cfg': lambda param: add_termination_pose(servo_termination_config, self.servo_pose_msg),
             'outcome': lambda param: send_end_fts_outcome_request(param)},
            {'step_name': 'reengage_brick',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.place_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(engage_termination_config, self.place_pose_msg)},
            {'step_name': 'release_brick',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.release_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(release_termination_config, self.release_pose_msg)},
            {'step_name': 'disengage_brick',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.disengage_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.disengage_pose_msg)}
        ]

    def execute_skill(self, execution_params, skill_params) -> Tuple[List[TerminationSignal], List[int]]:
        # QOL 
        if 'T_lego_world' not in skill_params:
            print(f"PlaceLegoSkill expects target brick pose transform: skill_params['T_lego_world'] = RigidTransform()")
        if 'approach_height_offset' not in skill_params:
            print(f"PlaceLegoSkill expects an approach height offset (meters): skill_params['approach_height_offset'] = float(0.010 m)")
        if 'place_rotation' not in skill_params:
            print(f"PlaceLegoSkill expects a release rotation (deg): skill_params['place_rotation'] = float(30 deg)")
        if 'place_perturbation' not in skill_params:
            self.place_perturbation = np.random.uniform(0.000, 0.001, size=(3,))
            print(f"No initial place perturbation specified, using random perturbation: {tuple(self.place_perturbation)} [m]")
        else:
            self.place_perturbation = np.array(skill_params['place_perturbation'])

        self.T_lego_world = skill_params['T_lego_world']
        self.approach_height_offset = skill_params['approach_height_offset']
        self.place_rotation = skill_params['place_rotation']

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
        # A lego stud is 1.7 mm tall
        self.T_lego_servo = RigidTransform(
            translation=np.array([0.0, 0.0, -0.010]),
            from_frame='lego', to_frame='lego'
        )
        self.T_lego_disengage = RigidTransform(

        )

        self.T_lego_place_bad = RigidTransform(translation=self.place_perturbation,
                                               from_frame='lego', to_frame='lego')

        self.T_place_target = self.T_lego_world.copy()
        
        self.place_pose = self.T_place_target * self.T_lego_ee.inverse()
        self.place_pose_msg = self.place_pose.pose_msg

        self.place_pose_bad = self.T_place_target * self.T_lego_place_bad * self.T_lego_ee.inverse()
        self.place_pose_bad_msg = self.place_pose_bad.pose_msg

        self.servo_pose_bad = self.T_place_target * self.T_lego_place_bad * self.T_lego_servo * self.T_lego_ee.inverse()
        self.servo_pose_bad_msg = self.servo_pose_bad.pose_msg

        self.approach_pose = self.T_place_target * self.T_lego_approach * self.T_lego_ee.inverse()
        self.approach_pose_msg = self.approach_pose.pose_msg

        self.release_pose = self.T_place_target * self.T_lego_rotation * self.T_lego_rotation_point_offset.inverse() * self.T_lego_ee.inverse()
        self.release_pose_msg = self.release_pose.pose_msg

        self.servo_pose = self.T_place_target * self.T_lego_servo * self.T_lego_ee.inverse()
        self.servo_pose_msg = self.servo_pose.pose_msg

        self.disengage_pose = self.T_place_target * self.T_lego_approach
        self.disengage_pose *= self.T_lego_rotation * self.T_lego_rotation_point_offset.inverse() * self.T_lego_ee.inverse()
        self.disengage_pose_msg = self.disengage_pose.pose_msg

        return super().execute_skill(execution_params)

    

class PlaceLegoSkill(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, namespace: str, params=None):

        super().__init__(robot_commander, namespace, params)

        if 'T_lego_ee' not in self.params:
            print(f"PlaceLegoHardcodedCorrectionSkill expects end effector transform: params['T_lego_ee'] = RigidTransform()")

        self.T_lego_ee = self.params['T_lego_ee']

        self.skill_steps = [
            {'step_name': 'go_to_approach_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.approach_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.approach_pose_msg)},
            {'step_name': 'engage_brick',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.place_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(engage_termination_config, self.place_pose_msg)},
            {'step_name': 'pull_up',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.servo_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(servo_termination_config, self.servo_pose_msg),
             'outcome': lambda param: send_end_fts_outcome_request(param)},
            {'step_name': 'reengage_brick',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.place_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(engage_termination_config, self.place_pose_msg)},
            {'step_name': 'release_brick',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.release_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(release_termination_config, self.release_pose_msg)},
            {'step_name': 'disengage_brick',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.disengage_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.disengage_pose_msg)}
        ]

    def execute_skill(self, execution_params, skill_params) -> Tuple[List[TerminationSignal], List[int]]:
        # QOL 
        if 'T_lego_world' not in skill_params:
            print(f"PlaceLegoSkill expects target brick pose transform: skill_params['T_lego_world'] = RigidTransform()")
        if 'approach_height_offset' not in skill_params:
            print(f"PlaceLegoSkill expects an approach height offset (meters): skill_params['approach_height_offset'] = float(0.010 m)")
        if 'place_rotation' not in skill_params:
            print(f"PlaceLegoSkill expects a release rotation (deg): skill_params['place_rotation'] = float(30 deg)")

        self.T_lego_world = skill_params['T_lego_world']
        self.approach_height_offset = skill_params['approach_height_offset']
        self.place_rotation = skill_params['place_rotation']

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
        self.T_lego_disengage = RigidTransform(

        )


        self.T_place_target = self.T_lego_world.copy()
        if skill_params['place_perturbation']:
            self.place_perturbation = np.array(skill_params['place_perturbation'])
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

        self.disengage_pose = self.T_place_target * self.T_lego_approach
        self.disengage_pose *= self.T_lego_rotation * self.T_lego_rotation_point_offset.inverse() * self.T_lego_ee.inverse()
        self.disengage_pose_msg = self.disengage_pose.pose_msg

        return super().execute_skill(execution_params)

class PickLegoHardcodedCorrectionSkill(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, namespace: str, params=None):

        super().__init__(robot_commander, namespace, params)

        if 'T_lego_ee' not in self.params:
            print(f"PlaceLegoHardcodedCorrectionSkill expects end effector transform: params['T_lego_ee'] = RigidTransform()")

        self.T_lego_ee = self.params['T_lego_ee']

        self.skill_steps = [
            {'step_name': 'go_to_approach_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.approach_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.approach_pose_msg)},
            {'step_name': 'engage_brick_bad',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.place_pose_bad_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(engage_termination_config, self.place_pose_bad_msg)},
            {'step_name': 'pull_up',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.servo_pose_bad_msg, wait=False, velocity_scaling=0.001),
             'termination_cfg': lambda param: add_termination_pose(servo_termination_config, self.servo_pose_bad_msg),
             'outcome': lambda param: send_end_fts_outcome_request(param)},
            {'step_name': 'reset_to_approach_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.approach_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.approach_pose_msg)},
            {'step_name': 'engage_brick_good',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.place_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(engage_termination_config, self.place_pose_msg)},
            {'step_name': 'pull_up',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.servo_pose_msg, wait=False, velocity_scaling=0.001),
             'termination_cfg': lambda param: add_termination_pose(servo_termination_config, self.servo_pose_msg),
             'outcome': lambda param: send_end_fts_outcome_request(param)},
            {'step_name': 'reengage_brick',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.place_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(engage_termination_config, self.place_pose_msg)},
            {'step_name': 'release_brick',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.release_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(release_termination_config, self.release_pose_msg)},
            {'step_name': 'disengage_brick',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.disengage_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.disengage_pose_msg)}
        ]

    def execute_skill(self, execution_params, skill_params) -> Tuple[List[TerminationSignal], List[int]]:
        # QOL 
        if 'T_lego_world' not in skill_params:
            print(f"PlaceLegoSkill expects target brick pose transform: skill_params['T_lego_world'] = RigidTransform()")
        if 'approach_height_offset' not in skill_params:
            print(f"PlaceLegoSkill expects an approach height offset (meters): skill_params['approach_height_offset'] = float(0.010 m)")
        if 'place_rotation' not in skill_params:
            print(f"PlaceLegoSkill expects a release rotation (deg): skill_params['place_rotation'] = float(30 deg)")
        if 'place_perturbation' not in skill_params:
            self.place_perturbation = np.random.uniform(0.000, 0.001, size=(3,))
            print(f"No initial place perturbation specified, using random perturbation: {tuple(self.place_perturbation)} [m]")
        else:
            self.place_perturbation = np.array(skill_params['place_perturbation'])

        self.T_lego_world = skill_params['T_lego_world']
        self.approach_height_offset = skill_params['approach_height_offset']
        self.place_rotation = skill_params['place_rotation']

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
        # A lego stud is 1.7 mm tall
        self.T_lego_servo = RigidTransform(
            translation=np.array([0.0, 0.0, -0.010]),
            from_frame='lego', to_frame='lego'
        )
        self.T_lego_disengage = RigidTransform(

        )

        self.T_lego_place_bad = RigidTransform(translation=self.place_perturbation,
                                               from_frame='lego', to_frame='lego')

        self.T_place_target = self.T_lego_world.copy()
        
        self.place_pose = self.T_place_target * self.T_lego_ee.inverse()
        self.place_pose_msg = self.place_pose.pose_msg

        self.place_pose_bad = self.T_place_target * self.T_lego_place_bad * self.T_lego_ee.inverse()
        self.place_pose_bad_msg = self.place_pose_bad.pose_msg

        self.servo_pose_bad = self.T_place_target * self.T_lego_place_bad * self.T_lego_servo * self.T_lego_ee.inverse()
        self.servo_pose_bad_msg = self.servo_pose_bad.pose_msg

        self.approach_pose = self.T_place_target * self.T_lego_approach * self.T_lego_ee.inverse()
        self.approach_pose_msg = self.approach_pose.pose_msg

        self.release_pose = self.T_place_target * self.T_lego_rotation * self.T_lego_rotation_point_offset.inverse() * self.T_lego_ee.inverse()
        self.release_pose_msg = self.release_pose.pose_msg

        self.servo_pose = self.T_place_target * self.T_lego_servo * self.T_lego_ee.inverse()
        self.servo_pose_msg = self.servo_pose.pose_msg

        self.disengage_pose = self.T_place_target * self.T_lego_approach
        self.disengage_pose *= self.T_lego_rotation * self.T_lego_rotation_point_offset.inverse() * self.T_lego_ee.inverse()
        self.disengage_pose_msg = self.disengage_pose.pose_msg

        return super().execute_skill(execution_params)

class PickLegoSkill(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, namespace: str, params=None):

        super().__init__(robot_commander, namespace, params)

        if 'T_lego_ee' not in self.params:
            print(f"PlaceLegoHardcodedCorrectionSkill expects end effector transform: params['T_lego_ee'] = RigidTransform()")

        self.T_lego_ee = self.params['T_lego_ee']

        self.skill_steps = [
            {'step_name': 'go_to_approach_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.approach_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.approach_pose_msg)},
            {'step_name': 'engage_brick',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.place_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(engage_termination_config, self.place_pose_msg)},
            {'step_name': 'pull_up',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.servo_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(servo_termination_config, self.servo_pose_msg),
             'outcome': lambda param: send_end_fts_outcome_request(param)},
            {'step_name': 'reengage_brick',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.place_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(engage_termination_config, self.place_pose_msg)},
            {'step_name': 'release_brick',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.release_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(release_termination_config, self.release_pose_msg)},
            {'step_name': 'disengage_brick',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.disengage_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.disengage_pose_msg)}
        ]

    def execute_skill(self, execution_params, skill_params) -> Tuple[List[TerminationSignal], List[int]]:
        # QOL 
        if 'T_lego_world' not in skill_params:
            print(f"PlaceLegoSkill expects target brick pose transform: skill_params['T_lego_world'] = RigidTransform()")
        if 'approach_height_offset' not in skill_params:
            print(f"PlaceLegoSkill expects an approach height offset (meters): skill_params['approach_height_offset'] = float(0.010 m)")
        if 'place_rotation' not in skill_params:
            print(f"PlaceLegoSkill expects a release rotation (deg): skill_params['place_rotation'] = float(30 deg)")

        self.T_lego_world = skill_params['T_lego_world']
        self.approach_height_offset = skill_params['approach_height_offset']
        self.place_rotation = skill_params['place_rotation']

        self.T_lego_approach = RigidTransform(
            translation=np.array([0.0, 0.0, -abs(self.approach_height_offset)]),
            from_frame='lego', to_frame='lego'
        )
        self.T_lego_rotation = RigidTransform(
            rotation=RigidTransform.y_axis_rotation(np.deg2rad(self.place_rotation)), 
            translation=np.array([LEGO_BLOCK_WIDTH/2,0,LEGO_BLOCK_HEIGHT]), 
            from_frame='lego', to_frame='lego'
        )
        self.T_lego_rotation_point_offset = RigidTransform(
            translation=np.array([LEGO_BLOCK_WIDTH/2,0,LEGO_BLOCK_HEIGHT]), 
            from_frame='lego', to_frame='lego'
        )
        self.T_lego_servo = RigidTransform(
            translation=np.array([0.0, 0.0, -0.001]),
            from_frame='lego', to_frame='lego'
        )
        self.T_lego_disengage = RigidTransform(

        )


        self.T_place_target = self.T_lego_world.copy()
        if skill_params['place_perturbation']:
            self.place_perturbation = np.array(skill_params['place_perturbation'])
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

        self.disengage_pose = self.T_place_target * self.T_lego_approach
        self.disengage_pose *= self.T_lego_rotation * self.T_lego_rotation_point_offset.inverse() * self.T_lego_ee.inverse()
        self.disengage_pose_msg = self.disengage_pose.pose_msg

        return super().execute_skill(execution_params)

class MoveToAbovePerturbLegoPose(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, namespace: str, params=None):

        super().__init__(robot_commander, namespace, params)

        if 'T_lego_ee' not in self.params:
            print(f"MoveToAbovePerturbLegoPose expects end effector transform: params['T_lego_ee'] = RigidTransform()")

        self.T_lego_ee = self.params['T_lego_ee']

        self.skill_steps = [
            {'step_name': 'go_to_approach_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.approach_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.approach_pose_msg)},
        ]

    def execute_skill(self, execution_params, skill_params) -> Tuple[List[TerminationSignal], List[int]]:
        # QOL 
        if 'T_lego_world' not in skill_params:
            print(f"MoveToAbovePerturbLegoPose expects target brick pose transform: skill_params['T_lego_world'] = RigidTransform()")
        if 'approach_height_offset' not in skill_params:
            print(f"MoveToAbovePerturbLegoPose expects an approach height offset (meters): skill_params['approach_height_offset'] = float(0.010 m)")
        if 'place_perturbation' not in skill_params:
            self.place_perturbation = np.random.uniform(-0.001, 0.001, size=(3,))
            print(f"No initial place perturbation specified, using random perturbation: {tuple(self.place_perturbation)} [m]")
        else:
            self.place_perturbation = np.array(skill_params['place_perturbation'])

        self.T_lego_world = skill_params['T_lego_world']
        self.approach_height_offset = skill_params['approach_height_offset']

        self.T_place_target = self.T_lego_world.copy()

        self.T_lego_approach = RigidTransform(
            translation=np.array([0.0, 0.0, -abs(self.approach_height_offset)]),
            from_frame='lego', to_frame='lego'
        )

        self.place_perturb_pose = RigidTransform(translation=[self.place_perturbation[0], self.place_perturbation[1], 0.0],
                                                 rotation=RigidTransform.z_axis_rotation(np.deg2rad(self.place_perturbation[2])),
                                                 from_frame='lego', to_frame='lego')

        self.approach_pose = self.T_place_target * self.place_perturb_pose * self.T_lego_approach * self.T_lego_ee.inverse()
        self.approach_pose_msg = self.approach_pose.pose_msg

        return super().execute_skill(execution_params)


class MoveToPerturbLegoPose(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, namespace: str, params=None):

        super().__init__(robot_commander, namespace, params)

        if 'T_lego_ee' not in self.params:
            print(f"MoveToPerturbLegoPose expects end effector transform: params['T_lego_ee'] = RigidTransform()")

        self.T_lego_ee = self.params['T_lego_ee']

        self.skill_steps = [
            {'step_name': 'go_to_approach_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.approach_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.approach_pose_msg)},
            {'step_name': 'go_to_perturb_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.place_perturb_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(engage_termination_config, self.place_perturb_pose_msg)},
        ]

    def execute_skill(self, execution_params, skill_params) -> Tuple[List[TerminationSignal], List[int]]:
        # QOL 
        if 'T_lego_world' not in skill_params:
            print(f"MoveToPerturbLegoPose expects target brick pose transform: skill_params['T_lego_world'] = RigidTransform()")
        if 'approach_height_offset' not in skill_params:
            print(f"MoveToPerturbLegoPose expects an approach height offset (meters): skill_params['approach_height_offset'] = float(0.010 m)")
        if 'place_perturbation' not in skill_params:
            self.place_perturbation = np.random.uniform(-0.001, 0.001, size=(3,))
            print(f"No initial place perturbation specified, using random perturbation: {tuple(self.place_perturbation)} [m]")
        else:
            self.place_perturbation = np.array(skill_params['place_perturbation'])

        self.T_lego_world = skill_params['T_lego_world']
        self.approach_height_offset = skill_params['approach_height_offset']

        self.T_place_target = self.T_lego_world.copy()

        self.T_lego_approach = RigidTransform(
            translation=np.array([0.0, 0.0, -abs(self.approach_height_offset)]),
            from_frame='lego', to_frame='lego'
        )

        self.place_perturb_pose = RigidTransform(translation=[self.place_perturbation[0], self.place_perturbation[1], 0.0],
                                                 rotation=RigidTransform.z_axis_rotation(np.deg2rad(self.place_perturbation[2])),
                                                 from_frame='lego', to_frame='lego')

        self.approach_pose = self.T_place_target * self.place_perturb_pose * self.T_lego_approach * self.T_lego_ee.inverse()
        self.approach_pose_msg = self.approach_pose.pose_msg

        self.place_perturb_pose = self.T_place_target * self.place_perturb_pose * self.T_lego_ee.inverse()
        self.place_perturb_pose_msg = self.place_perturb_pose.pose_msg

        return super().execute_skill(execution_params)

class MoveToLegoPose(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, namespace: str, params=None):

        super().__init__(robot_commander, namespace, params)

        if 'T_lego_ee' not in self.params:
            print(f"MoveToLegoPose expects end effector transform: params['T_lego_ee'] = RigidTransform()")

        self.T_lego_ee = self.params['T_lego_ee']

        self.skill_steps = [
            {'step_name': 'go_to_approach_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.approach_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.approach_pose_msg)},
            {'step_name': 'go_to_lego_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.lego_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(engage_termination_config, self.lego_pose_msg)},
        ]

    def execute_skill(self, execution_params, skill_params) -> Tuple[List[TerminationSignal], List[int]]:
        # QOL 
        if 'T_lego_world' not in skill_params:
            print(f"MoveToLegoPose expects target brick pose transform: skill_params['T_lego_world'] = RigidTransform()")
        if 'approach_height_offset' not in skill_params:
            print(f"MoveToLegoPose expects an approach height offset (meters): skill_params['approach_height_offset'] = float(0.010 m)")

        self.T_lego_world = skill_params['T_lego_world']
        self.approach_height_offset = skill_params['approach_height_offset']

        self.T_place_target = self.T_lego_world.copy()

        self.T_lego_approach = RigidTransform(
            translation=np.array([0.0, 0.0, -abs(self.approach_height_offset)]),
            from_frame='lego', to_frame='lego'
        )

        self.approach_pose = self.T_place_target * self.T_lego_approach * self.T_lego_ee.inverse()
        self.approach_pose_msg = self.approach_pose.pose_msg


        self.lego_pose = self.T_place_target * self.T_lego_ee.inverse()
        self.lego_pose_msg = self.lego_pose.pose_msg

        return super().execute_skill(execution_params)

class MoveToAboveLegoPose(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, namespace: str, params=None):

        super().__init__(robot_commander, namespace, params)

        if 'T_lego_ee' not in self.params:
            print(f"MoveToAboveLegoPose expects end effector transform: params['T_lego_ee'] = RigidTransform()")

        self.T_lego_ee = self.params['T_lego_ee']

        self.skill_steps = [
            {'step_name': 'go_to_approach_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.approach_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.approach_pose_msg)},
        ]

    def execute_skill(self, execution_params, skill_params) -> Tuple[List[TerminationSignal], List[int]]:
        # QOL 
        if 'T_lego_world' not in skill_params:
            print(f"MoveToAboveLegoPose expects target brick pose transform: skill_params['T_lego_world'] = RigidTransform()")
        if 'approach_height_offset' not in skill_params:
            print(f"MoveToAboveLegoPose expects an approach height offset (meters): skill_params['approach_height_offset'] = float(0.010 m)")

        self.T_lego_world = skill_params['T_lego_world']
        self.approach_height_offset = skill_params['approach_height_offset']

        self.T_place_target = self.T_lego_world.copy()

        self.T_lego_approach = RigidTransform(
            translation=np.array([0.0, 0.0, -abs(self.approach_height_offset)]),
            from_frame='lego', to_frame='lego'
        )

        self.approach_pose = self.T_place_target * self.T_lego_approach * self.T_lego_ee.inverse()
        self.approach_pose_msg = self.approach_pose.pose_msg

        return super().execute_skill(execution_params)

class PullUp(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, namespace: str, params=None):

        super().__init__(robot_commander, namespace, params)

        self.skill_steps = [
            {'step_name': 'pull_up',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.pull_up_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(servo_termination_config, self.pull_up_pose_msg),
             'outcome': lambda param: send_end_fts_outcome_request(param)},
        ]

    def execute_skill(self, execution_params, skill_params) -> Tuple[List[TerminationSignal], List[int]]:
        # QOL 
        if 'lift_height_offset' not in skill_params:
            print(f"PullUp expects an lift height offset (meters): skill_params['lift_height_offset'] = float(0.001 m)")

        current_pose_msg = self.robot_commander.get_current_pose()
        current_pose = RigidTransform.from_pose_msg(current_pose_msg, from_frame='ee')


        self.lift_height_offset = skill_params['lift_height_offset']

        self.T_pull_up = RigidTransform(
            translation=np.array([0.0, 0.0, -abs(self.lift_height_offset)]),
            from_frame='ee', to_frame='ee'
        )

        self.pull_up_pose = current_pose * self.T_pull_up
        self.pull_up_pose_msg = self.pull_up_pose.pose_msg

        return super().execute_skill(execution_params)

class MoveDown(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, namespace: str, params=None):

        super().__init__(robot_commander, namespace, params)

        self.skill_steps = [
            {'step_name': 'move_down',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.move_down_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(engage_termination_config, self.move_down_pose_msg)},
        ]

    def execute_skill(self, execution_params, skill_params) -> Tuple[List[TerminationSignal], List[int]]:
        # QOL 
        if 'height_offset' not in skill_params:
            print(f"MoveDown expects a height offset (meters): skill_params['height_offset'] = float(0.001 m)")

        current_pose_msg = self.robot_commander.get_current_pose()
        current_pose = RigidTransform.from_pose_msg(current_pose_msg, from_frame='ee')


        self.height_offset = skill_params['height_offset']

        self.T_move_down = RigidTransform(
            translation=np.array([0.0, 0.0, abs(self.height_offset)]),
            from_frame='ee', to_frame='ee'
        )

        self.move_down_pose = current_pose * self.T_move_down
        self.move_down_pose_msg = self.move_down_pose.pose_msg

        return super().execute_skill(execution_params)


class PlaceLego(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, namespace: str, params=None):

        super().__init__(robot_commander, namespace, params)

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

    def __init__(self, robot_commander: BaseRobotCommander, namespace: str, params=None):

        super().__init__(robot_commander, namespace, params)

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