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

### Termination Configs for Connector Skills ###
# Rapid motion
rapid_termination_config = {
    'time': {'duration': 20.0},
    'pose': {'pos_tolerance': 0.001,
                'orient_tolerance': 0.01,
                'pose': None},
}
# Engaging a connector on the build plate
# Terminate if Fz exceeds force to join connectors with good alignment
# 10N is an estimate
z_engage_termination_config = {
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
                    'z': [-10, 10]},
                'torque': {
                    'x': [-2, 2],
                    'y': [-2, 2],
                    'z': [-2, 2]}
            }}
}

# Releasing a connector through rotation
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
# Pulling up on a connector to check if connected
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


class MoveToAbovePerturbConnectorPose(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, gripper_controller: BaseGripperController, namespace: str, params=None):

        super().__init__(robot_commander, gripper_controller, namespace, params)

        if 'T_hande_ee' not in self.params:
            print(f"MoveToAbovePerturbConnectorPose expects end effector transform: params['T_hande_ee'] = RigidTransform()")

        self.T_hande_ee = self.params['T_hande_ee']

        self.skill_steps = [
            {'step_name': 'go_to_approach_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.approach_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.approach_pose_msg)},
        ]

    def execute_skill(self, execution_params, skill_params) -> Tuple[List[TerminationSignal], List[int]]:
        # QOL 
        if 'T_connector_world' not in skill_params:
            print(f"MoveToAbovePerturbConnectorPose expects target connector pose transform: skill_params['T_connector_world'] = RigidTransform()")
        if 'approach_height_offset' not in skill_params:
            print(f"MoveToAbovePerturbConnectorPose expects an approach height offset (meters): skill_params['approach_height_offset'] = float(0.010 m)")
        if 'place_perturbation' not in skill_params:
            self.place_perturbation = np.random.uniform(-0.001, 0.001, size=(3,))
            print(f"No initial place perturbation specified, using random perturbation: {tuple(self.place_perturbation)} [m]")
        else:
            self.place_perturbation = np.array(skill_params['place_perturbation'])

        self.T_connector_world = skill_params['T_connector_world']
        self.approach_height_offset = skill_params['approach_height_offset']

        self.T_place_target = self.T_connector_world.copy()

        self.T_connector_approach = RigidTransform(
            translation=np.array([0.0, 0.0, -abs(self.approach_height_offset)]),
            from_frame='hande', to_frame='hande'
        )

        self.place_perturb_pose = RigidTransform(translation=[self.place_perturbation[0], self.place_perturbation[1], 0.0],
                                                 rotation=RigidTransform.z_axis_rotation(np.deg2rad(self.place_perturbation[2])),
                                                 from_frame='hande', to_frame='hande')

        self.approach_pose = self.T_place_target * self.place_perturb_pose * self.T_connector_approach * self.T_hande_ee.inverse()
        self.approach_pose_msg = self.approach_pose.pose_msg

        return super().execute_skill(execution_params)

class MoveToAboveConnectorPose(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, gripper_controller: BaseGripperController, namespace: str, params=None):

        super().__init__(robot_commander, gripper_controller, namespace, params)

        if 'T_hande_ee' not in self.params:
            print(f"MoveToAboveConnectorPose expects end effector transform: params['T_hande_ee'] = RigidTransform()")

        self.T_hande_ee = self.params['T_hande_ee']

        self.skill_steps = [
            {'step_name': 'go_to_approach_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.approach_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.approach_pose_msg)},
        ]

    def execute_skill(self, execution_params, skill_params) -> Tuple[List[TerminationSignal], List[int]]:
        # QOL 
        if 'T_connector_world' not in skill_params:
            print(f"MoveToAboveConnectorPose expects target connector pose transform: skill_params['T_connector_world'] = RigidTransform()")
        if 'approach_height_offset' not in skill_params:
            print(f"MoveToAboveConnectorPose expects an approach height offset (meters): skill_params['approach_height_offset'] = float(0.010 m)")

        self.T_connector_world = skill_params['T_connector_world']
        self.approach_height_offset = skill_params['approach_height_offset']

        self.T_place_target = self.T_connector_world.copy()

        self.T_connector_approach = RigidTransform(
            translation=np.array([0.0, 0.0, -abs(self.approach_height_offset)]),
            from_frame='hande', to_frame='hande'
        )

        self.approach_pose = self.T_place_target * self.T_connector_approach * self.T_hande_ee.inverse()
        self.approach_pose_msg = self.approach_pose.pose_msg

        return super().execute_skill(execution_params)

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

class MoveUp(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, gripper_controller: BaseGripperController, namespace: str, params=None):

        super().__init__(robot_commander, gripper_controller, namespace, params)

        self.skill_steps = [
            {'step_name': 'move_up',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.move_up_pose_msg, wait=False, velocity_scaling=self.velocity_scaling),
             'termination_cfg': lambda param: add_termination_pose(servo_termination_config, self.move_up_pose_msg)},
        ]

    def execute_skill(self, execution_params, skill_params) -> Tuple[List[TerminationSignal], List[int]]:
        # QOL 
        if 'lift_height_offset' not in skill_params:
            print(f"moveUp expects an lift height offset (meters): skill_params['lift_height_offset'] = float(0.001 m)")
        if 'velocity_scaling' not in skill_params:
            self.velocity_scaling = 0.01
            print(f"No initial velocity scaling specified, using: {self.velocity_scaling}")
        else:
            self.velocity_scaling = skill_params['velocity_scaling']

        current_pose_msg = self.robot_commander.get_current_pose()
        current_pose = RigidTransform.from_pose_msg(current_pose_msg, from_frame='ee')


        self.lift_height_offset = skill_params['lift_height_offset']

        self.T_move_up = RigidTransform(
            translation=np.array([0.0, 0.0, -abs(self.lift_height_offset)]),
            from_frame='ee', to_frame='ee'
        )

        self.move_up_pose = current_pose * self.T_move_up
        self.move_up_pose_msg = self.move_up_pose.pose_msg

        return super().execute_skill(execution_params)

class MoveDown(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, gripper_controller: BaseGripperController, namespace: str, params=None):

        super().__init__(robot_commander, gripper_controller, namespace, params)

        self.skill_steps = [
            {'step_name': 'move_down',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.move_down_pose_msg, wait=False, velocity_scaling=self.velocity_scaling),
             'termination_cfg': lambda param: add_termination_pose(z_engage_termination_config, self.move_down_pose_msg)},
        ]

    def execute_skill(self, execution_params, skill_params) -> Tuple[List[TerminationSignal], List[int]]:
        # QOL 
        if 'height_offset' not in skill_params:
            print(f"MoveDown expects a height offset (meters): skill_params['height_offset'] = float(0.001 m)")
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

class Move2D(BaseSkill):

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

class PickConnector(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, gripper_controller: BaseGripperController, namespace: str, params=None):

        super().__init__(robot_commander, gripper_controller, namespace, params)

        if 'T_hande_ee' not in self.params:
            print(f"PickConnector expects end effector transform: params['T_hande_ee'] = RigidTransform()")

        self.T_hande_ee = self.params['T_hande_ee']

        if 'T_connector_world' not in self.params:
            print(f"PickConnector expects target connector pose transform: params['T_connector_world'] = RigidTransform()")

        self.T_connector_world = self.params['T_connector_world']

        if 'approach_height_offset' not in self.params:
            print(f"PickConnector expects an approach height offset (meters): params['approach_height_offset'] = float(0.010 m)")

        self.approach_height_offset = self.params['approach_height_offset']

        self.connector_pose = self.T_connector_world.copy()
        self.robot_connector_pose = self.connector_pose * self.T_hande_ee.inverse()
        self.connector_pose_msg = self.robot_connector_pose.pose_msg

        self.T_connector_approach = RigidTransform(
            translation=np.array([0.0, 0.0, -abs(self.approach_height_offset)]),
            from_frame='hande', to_frame='hande'
        )

        self.approach_pose = self.connector_pose * self.T_connector_approach * self.T_hande_ee.inverse()
        self.approach_pose_msg = self.approach_pose.pose_msg

        self.skill_steps = [
            {'step_name': 'go_to_approach_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.approach_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.approach_pose_msg)},
            {'step_name': 'go_to_connector_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.connector_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.connector_pose_msg)},
            {'step_name': 'close_gripper',
             'robot_command': lambda param: self.gripper_controller.close(force=160),
             'termination_cfg': None},
            {'step_name': 'go_to_approach_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.approach_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.approach_pose_msg)},
        ]


class PlaceConnector(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, gripper_controller: BaseGripperController, namespace: str, params=None):

        super().__init__(robot_commander, gripper_controller, namespace, params)

        if 'T_hande_ee' not in self.params:
            print(f"PlaceConnector expects end effector transform: params['T_hande_ee'] = RigidTransform()")

        self.T_hande_ee = self.params['T_hande_ee']

        if 'T_connector_world' not in self.params:
            print(f"PlaceConnector expects target connector pose transform: params['T_connector_world'] = RigidTransform()")

        self.T_connector_world = self.params['T_connector_world']

        if 'approach_height_offset' not in self.params:
            print(f"PlaceConnector expects an approach height offset (meters): params['approach_height_offset'] = float(0.010 m)")

        self.approach_height_offset = self.params['approach_height_offset']

        if 'reset_x_offset' not in self.params:
            print(f"PlaceConnector expects a reset x offset (meters): params['reset_x_offset'] = float(0.010 m)")

        self.reset_x_offset = self.params['reset_x_offset']

        self.connector_pose = self.T_connector_world.copy()
        self.robot_connector_pose = self.connector_pose * self.T_hande_ee.inverse()
        self.connector_pose_msg = self.robot_connector_pose.pose_msg

        self.T_connector_approach = RigidTransform(
            translation=np.array([0.0, 0.0, -abs(self.approach_height_offset)]),
            from_frame='hande', to_frame='hande'
        )

        self.T_connector_reset_x = RigidTransform(
            translation=np.array([0.0, abs(self.reset_x_offset), 0.0]),
            from_frame='hande', to_frame='hande'
        )

        self.T_connector_reset_neg_x = RigidTransform(
            translation=np.array([0.0, -abs(self.reset_x_offset), 0.0]),
            from_frame='hande', to_frame='hande'
        )

        self.approach_pose = self.connector_pose * self.T_connector_approach * self.T_hande_ee.inverse()
        self.approach_pose_msg = self.approach_pose.pose_msg

        self.reset_x_pose = self.connector_pose * self.T_connector_reset_x * self.T_hande_ee.inverse()
        self.reset_x_pose_msg = self.reset_x_pose.pose_msg

        self.reset_neg_x_pose = self.connector_pose * self.T_connector_reset_neg_x * self.T_hande_ee.inverse()
        self.reset_neg_x_pose_msg = self.reset_neg_x_pose.pose_msg


        self.skill_steps = [
            {'step_name': 'go_to_approach_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.approach_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.approach_pose_msg)},
            {'step_name': 'go_to_connector_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.connector_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.connector_pose_msg)},
            {'step_name': 'open_gripper',
             'robot_command': lambda param: self.gripper_controller.open(),
             'termination_cfg': None},
            {'step_name': 'go_to_reset_x_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.reset_x_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(engage_termination_config, self.reset_x_pose_msg)},
            {'step_name': 'go_to_reset_neg_x_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.reset_neg_x_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(engage_termination_config, self.reset_x_pose_msg)},
            {'step_name': 'go_to_connector_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.connector_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.connector_pose_msg)},
            {'step_name': 'go_to_approach_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.approach_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.approach_pose_msg)},
        ]

class ResetConnector(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, gripper_controller: BaseGripperController, namespace: str, params=None):

        super().__init__(robot_commander, gripper_controller, namespace, params)

        if 'T_hande_ee' not in self.params:
            print(f"ResetConnector expects end effector transform: params['T_hande_ee'] = RigidTransform()")

        self.T_hande_ee = self.params['T_hande_ee']

        if 'T_connector_world_reset_x' not in self.params:
            print(f"ResetConnector expects target connector pose transform: params['T_connector_world_reset_x'] = RigidTransform()")

        self.T_connector_world_reset_x = self.params['T_connector_world_reset_x']

        if 'T_connector_world_reset_y' not in self.params:
            print(f"ResetConnector expects target connector pose transform: params['T_connector_world_reset_y'] = RigidTransform()")

        self.T_connector_world_reset_y = self.params['T_connector_world_reset_y']

        if 'reset_x_offset' not in self.params:
            print(f"ResetConnector expects a reset x offset (meters): params['reset_x_offset'] = float(0.010 m)")

        self.reset_x_offset = self.params['reset_x_offset']

        if 'reset_y_offset' not in self.params:
            print(f"ResetConnector expects a reset y offset (meters): params['reset_y_offset'] = float(0.010 m)")

        self.reset_y_offset = self.params['reset_y_offset']

        if 'height_offset' not in self.params:
            print(f"ResetConnector expects an approach height offset (meters): params['approach_height_offset'] = float(0.010 m)")

        self.height_offset = self.params['height_offset']

        self.T_connector_approach_yz = RigidTransform(
            translation=np.array([-abs(self.reset_y_offset), 0.0, -abs(self.height_offset)]),
            from_frame='hande', to_frame='hande'
        )

        self.T_connector_approach_y = RigidTransform(
            translation=np.array([-abs(self.reset_y_offset), 0.0, 0.0]),
            from_frame='hande', to_frame='hande'
        )

        self.T_connector_approach_xz = RigidTransform(
            translation=np.array([0.0, abs(self.reset_x_offset), -abs(self.height_offset)]),
            from_frame='hande', to_frame='hande'
        )

        self.T_connector_approach_x = RigidTransform(
            translation=np.array([0.0, abs(self.reset_x_offset), 0.0]),
            from_frame='hande', to_frame='hande'
        )

        

        self.reset_yz_approach_pose = self.T_connector_world_reset_y * self.T_connector_approach_yz * self.T_hande_ee.inverse()
        self.reset_yz_approach_pose_msg = self.reset_yz_approach_pose.pose_msg

        self.reset_y_approach_pose = self.T_connector_world_reset_y * self.T_connector_approach_y * self.T_hande_ee.inverse()
        self.reset_y_approach_pose_msg = self.reset_y_approach_pose.pose_msg

        self.reset_y_pose = self.T_connector_world_reset_y * self.T_hande_ee.inverse()
        self.reset_y_pose_msg = self.reset_y_pose.pose_msg

        self.reset_xz_approach_pose = self.T_connector_world_reset_x * self.T_connector_approach_xz * self.T_hande_ee.inverse()
        self.reset_xz_approach_pose_msg = self.reset_xz_approach_pose.pose_msg

        self.reset_x_approach_pose = self.T_connector_world_reset_x * self.T_connector_approach_x * self.T_hande_ee.inverse()
        self.reset_x_approach_pose_msg = self.reset_x_approach_pose.pose_msg

        self.reset_x_pose = self.T_connector_world_reset_x * self.T_hande_ee.inverse()
        self.reset_x_pose_msg = self.reset_x_pose.pose_msg

        self.skill_steps = [
            {'step_name': 'go_to_reset_yz_approach_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.reset_yz_approach_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(z_engage_termination_config, self.reset_yz_approach_pose_msg)},
            {'step_name': 'go_to_reset_y_approach_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.reset_y_approach_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(engage_termination_config, self.reset_y_approach_pose_msg)},
            {'step_name': 'go_to_reset_y_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.reset_y_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(engage_termination_config, self.reset_y_pose_msg)},
            {'step_name': 'go_to_reset_y_approach_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.reset_y_approach_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(engage_termination_config, self.reset_y_approach_pose_msg)},
            {'step_name': 'go_to_reset_yz_approach_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.reset_yz_approach_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(z_engage_termination_config, self.reset_yz_approach_pose_msg)},
            {'step_name': 'go_to_reset_xz_approach_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.reset_xz_approach_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(z_engage_termination_config, self.reset_xz_approach_pose_msg)},
            {'step_name': 'go_to_reset_x_approach_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.reset_x_approach_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(engage_termination_config, self.reset_x_approach_pose_msg)},
            {'step_name': 'go_to_reset_x_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.reset_x_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(engage_termination_config, self.reset_x_pose_msg)},
            {'step_name': 'go_to_reset_x_approach_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.reset_x_approach_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(engage_termination_config, self.reset_x_approach_pose_msg)},
            {'step_name': 'go_to_reset_xz_approach_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.reset_xz_approach_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(z_engage_termination_config, self.reset_xz_approach_pose_msg)},
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