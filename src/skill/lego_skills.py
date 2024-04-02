#!/usr/bin/env python3

import copy
import numpy as np
import subprocess
import signal
import os

from geometry_msgs.msg import Pose, Twist
from autolab_core import RigidTransform

from robot_controller.robot_commander import BaseRobotCommander
from skill.base_skill import BaseSkill
import terminator.utils as t_utils

from typing import Tuple, List

class PlaceLegoSkill(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander):

        super().__init__(robot_commander)

        self.skill_steps = [
             {'step_name': 'go_to_home_joints',
              'command': lambda: self.robot_commander.go_to_joints([0, 0, 0, 0, -np.pi/2, 0], wait=False),
              'termination_cfg': {'joint': {'positions': [0, 0, 0, 0, -np.pi/2, 0]}}}
        ]

class PickLegoSkill(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander):

        super().__init__(robot_commander)

        self.skill_steps = [
             {'step_name': 'go_to_home_joints',
              'command': lambda: self.robot_commander.go_to_joints([0, 0, 0, 0, -np.pi/2, 0], wait=False),
              'termination_cfg': {'joint': {'positions': [0, 0, 0, 0, -np.pi/2, 0]}}}
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