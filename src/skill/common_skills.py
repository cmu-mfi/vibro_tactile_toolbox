#!/usr/bin/env python3

import copy
import numpy as np

from geometry_msgs.msg import Pose
from autolab_core import RigidTransform
from skill.base_skill import BaseSkill
from robot_controller.base_robot_commander import BaseRobotCommander
from gripper_controller.base_gripper_controller import BaseGripperController
import terminator.utils as t_utils

from vibro_tactile_toolbox.msg import TerminationSignal

from typing import Tuple, List

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
z_engage_termination_config = {
    'time': {'duration': 10.0},
    'pose': {'pos_tolerance': 0.0001,
                'orient_tolerance': 0.01,
                'pose': None},
    'fts': {'check_rate_ns': 1E7,
            'threshold': {
                'force': {
                    'x': [-10, 10],
                    'y': [-10, 10],
                    'z': [0, 10]},
                'torque': {
                    'x': [-1, 1],
                    'y': [-1, 1],
                    'z': [-1, 1]}
            }},
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
                    'x': [-50, 50],               # any substantial reaction force in x-y
                    'y': [-50, 50],
                    'z': [-float('inf'), 2]},   # reaction force pulling away in z
                'torque': {
                    'x': [-3, 3],               # any substantial reaction torque
                    'y': [-3, 3],
                    'z': [-3, 3]}
            }}
}

servo_termination_config = {
    'time': {'duration': 10.0},
    'pose': {'pos_tolerance': 0.001,
                'orient_tolerance': 0.01,
                'pose': None},
    'fts': {'check_rate_ns': 1E7,
            'threshold': {
                'force': {
                    'x': [-30, 30],               # any substantial reaction force in x-y
                    'y': [-30, 30],
                    'z': [-float('inf'), 8]},   # reaction force pulling away in z
                'torque': {
                    'x': [-3, 3],               # any substantial reaction torque
                    'y': [-3, 3],
                    'z': [-3, 3]}
            }}
}

def add_termination_pose(termination_config, pose : Pose):
    t_cfg = copy.deepcopy(termination_config)
    t_cfg['pose']['pose'] = t_utils.pose_to_dict(pose)
    return t_cfg

def add_termination_pose_and_audio_model(termination_config, pose : Pose, model_path: str):
    t_cfg = copy.deepcopy(termination_config)
    t_cfg['pose']['pose'] = t_utils.pose_to_dict(pose)
    if model_path is not None:
        t_cfg['audio'] = {'check_rate_ns': 1E7,
                          'model_path': model_path,
                          'channels': [0,1,2,3]}
    return t_cfg

class PullUp(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, gripper_controller: BaseGripperController, namespace: str, params=None):

        super().__init__(robot_commander, gripper_controller, namespace, params)

        self.skill_steps = [
            {'step_name': 'pull_up',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.pull_up_pose_msg, wait=False, velocity_scaling=self.velocity_scaling),
             'termination_cfg': lambda param: add_termination_pose(servo_termination_config, self.pull_up_pose_msg)},
        ]

    def execute_skill(self, execution_params, skill_params) -> Tuple[List[TerminationSignal], List[int]]:
        # QOL 
        if 'lift_height_offset' not in skill_params:
            print(f"PullUp expects an lift height offset (meters): skill_params['lift_height_offset'] = float(0.001 m)")
        if 'velocity_scaling' not in skill_params:
            self.velocity_scaling = 0.001
            print(f"No initial velocity scaling specified, using: {self.velocity_scaling}")
        else:
            self.velocity_scaling = skill_params['velocity_scaling']

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

class MoveDownToContact(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, gripper_controller: BaseGripperController, namespace: str, params=None):

        super().__init__(robot_commander, gripper_controller, namespace, params)

        self.model_path = None

        self.skill_steps = [
            {'step_name': 'move_down',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.move_down_pose_msg, wait=False, velocity_scaling=self.velocity_scaling),
             'termination_cfg': lambda param: add_termination_pose_and_audio_model(z_engage_termination_config, self.move_down_pose_msg, self.model_path)}
        ]

    def execute_skill(self, execution_params, skill_params) -> Tuple[List[TerminationSignal], List[int]]:
        # QOL 
        if 'height_offset' not in skill_params:
            print(f"MoveDown expects a height offset (meters): skill_params['height_offset'] = float(0.001 m)")
        if 'model_path' in skill_params:
            self.model_path = skill_params['model_path']
        if 'velocity_scaling' not in skill_params:
            self.velocity_scaling = 0.01
            print(f"No initial velocity scaling specified, using: {self.velocity_scaling}")
        else:
            self.velocity_scaling = skill_params['velocity_scaling']

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

class GoHome(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, gripper_controller: BaseGripperController, namespace: str, params=None):

        super().__init__(robot_commander, gripper_controller, namespace, params)

        self.skill_steps = [
             {'step_name': 'go_to_home_joints',
              'robot_command': lambda param: self.robot_commander.go_to_joints([0, 0, 0, 0, -np.pi/2, 0], wait=False),
              'termination_cfg': lambda param: {'joint': {'position': [0, 0, 0, 0, -np.pi/2, 0]}}},
        ]

class OpenGripper(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, gripper_controller: BaseGripperController, namespace: str, params=None):

        super().__init__(robot_commander, gripper_controller, namespace, params)

        self.skill_steps = [
            {'step_name': 'open_gripper',
             'robot_command': lambda param: self.gripper_controller.open(),
             'termination_cfg': None},
        ]

class CloseGripper(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, gripper_controller: BaseGripperController, namespace: str, params=None):

        super().__init__(robot_commander, gripper_controller, namespace, params)
        self.force = 160

        self.skill_steps = [
            {'step_name': 'close_gripper',
             'robot_command': lambda param: self.gripper_controller.close(force=self.force),
             'termination_cfg': None},
        ]

    def execute_skill(self, execution_params, skill_params = None) -> Tuple[List[TerminationSignal], List[int]]:
        if skill_params is not None:
            self.force = skill_params['force']
        return super().execute_skill(execution_params)