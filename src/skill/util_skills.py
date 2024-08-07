#!/usr/bin/env python3

import numpy as np

from skill.base_skill import BaseSkill
from robot_controller.base_robot_commander import BaseRobotCommander
from gripper_controller.base_gripper_controller import BaseGripperController
import terminator.utils as t_utils

from typing import Tuple, List

class GoHomeSkill(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, gripper_controller: BaseGripperController, namespace: str, params=None):

        super().__init__(robot_commander, gripper_controller, namespace, params)

        self.skill_steps = [
             {'step_name': 'go_to_home_joints',
              'robot_command': lambda param: self.robot_commander.go_to_joints([0, 0, 0, 0, -np.pi/2, 0], wait=False),
              'termination_cfg': lambda param: {'joint': {'position': [0, 0, 0, 0, -np.pi/2, 0]}}},
        ]
