#!/usr/bin/env python3

import json
import numpy as np
from terminator.base_termination_handler import BaseTerminationHandler

from sensor_msgs.msg import JointState
from vibro_tactile_toolbox.msg import TerminationSignal, TerminationConfig

import terminator.utils as t_utils


class JointTerminationHandler(BaseTerminationHandler):
    def __init__(self):
        super().__init__()
        self.input_data_class = JointState
        self.check_rate_ns = 10E6 # 10 ms default
        self.tolerance = 0.001

        self.current_joints = JointState()
        self.current_joints.position = [0.0, 0.0, 0.0, 0.0, -np.pi/2, 0.0]

        # Termination condition: Joint State
        self.goal_joints = JointState()
        self.goal_joints.position = [0.0, 0.0, 0.0, 0.0, -np.pi/2, 0.0]

    def update_config(self, cfg: TerminationConfig):
        """
        Update the termination handler config:
        - id
        - check_rate_ns
        - tolerance
        - joints
        """
        cfg_jsons = cfg.cfg_json
        cfg_json = json.loads(cfg_jsons)
        if 'joint' in cfg_json:
            self.live = True
            self.id = cfg_json['id']
            joint_cfg = cfg_json['joint']
            if 'check_rate_ns' in joint_cfg:
                self.check_rate_ns = joint_cfg['check_rate_ns']
            if 'tolerance' in joint_cfg:
                self.tolerance = joint_cfg['tolerance']
            if 'position' in joint_cfg:
                self.goal_joints = t_utils.dict_to_joint_state(joint_cfg)
        else:
            self.live = False

    def update_input_data(self, input_signal: JointState):
        """
        Set the current JointState
        """
        self.current_joints = input_signal
    
    def get_termination_signal(self) -> TerminationSignal:
        """
        Create the termination signal and add causes based on the current JointState
        """
        terminate = False
        
        cause = ''

        for index in range(len(self.goal_joints.position)):
            if abs(self.current_joints.position[index] - self.goal_joints.position[index]) > self.tolerance:
                err = self.goal_joints.position[index] - self.current_joints.position[index]
                cause += f"Joint error ({err} rad) is greater than the tolerance ({self.tolerance} rad). "
                
        if cause == '':
            cause = 'Joint termination handler caused by: goal joints reached.'
            terminate = True
        
        termination_signal = TerminationSignal()
        termination_signal.id = self.id
        termination_signal.terminate = terminate
        termination_signal.cause = cause
        return termination_signal