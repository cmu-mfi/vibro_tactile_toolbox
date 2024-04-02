#!/usr/bin/env python3

import json
import math
import pyquaternion as pyq

from terminator.base_termination_handler import BaseTerminationHandler

from geometry_msgs.msg import PoseStamped, Pose
from vibro_tactile_toolbox.msg import TerminationSignal, TerminationConfig

import terminator.utils as t_utils


def pose_to_list(pose: Pose):
    return [pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

class PoseTerminationHandler(BaseTerminationHandler):
    def __init__(self):
        self.id = -1
        self.input_data_class = PoseStamped
        self.check_rate_ns = 10E6 # 10 ms default
        self.pos_tolerance = 0.001
        self.orient_tolerance = 0.01

        self.current_pose = Pose()
        self.current_pose.position.x = 0.29
        self.current_pose.position.y = 0.0
        self.current_pose.position.z = 0.533
        self.current_pose.orientation.x = -1.0
        self.current_pose.orientation.y = 0.0
        self.current_pose.orientation.z = 0.0
        self.current_pose.orientation.w = 0.0

        # Termination condition: Magnitude threshold
        self.goal_pose = Pose()
        self.goal_pose.position.x = 0.29
        self.goal_pose.position.y = 0.0
        self.goal_pose.position.z = 0.533
        self.goal_pose.orientation.x = -1.0
        self.goal_pose.orientation.y = 0.0
        self.goal_pose.orientation.z = 0.0
        self.goal_pose.orientation.w = 0.0


    def update_config(self, cfg: TerminationConfig):
        """
        Update the termination handler config:
        - id
        - check_rate_ns
        - pos_tolerance
        - orient_tolerance
        - pose
        """
        cfg_jsons = cfg.cfg_json
        cfg_json = json.loads(cfg_jsons)
        if 'pose' in cfg_json:
            self.id = cfg_json['id']
            pose_cfg = cfg_json['pose']
            if 'check_rate_ns' in pose_cfg:
                self.check_rate_ns = pose_cfg['check_rate_ns']
            if 'pos_tolerance' in pose_cfg:
                self.pos_tolerance = pose_cfg['pos_tolerance']
            if 'orient_tolerance' in pose_cfg:
                self.orient_tolerance = pose_cfg['orient_tolerance']
            if 'pose' in pose_cfg:
                self.goal_pose = t_utils.dict_to_pose(robot_cfg['pose'])

    
    def update_input_data(self, input_signal: PoseStamped):
        """
        Extract the Pose from the most recent PoseStamped reading
        """
        self.current_pose = input_signal.pose
    
    def get_termination_signal(self) -> TerminationSignal:
        """
        Create the termination signal and add causes based on the current pose
        """
        terminate = False
        cause = ''

        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(self.current_pose)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(self.goal_pose)
        # Euclidean distance
        d = math.dist((x1, y1, z1), (x0, y0, z0))
        # angle between orientations
        quat_1 = pyq.Quaternion(qx0, qy0, qz0, qw0)
        quat_2 = pyq.Quaternion(qx1, qy1, qz1, qw1)
        phi = 2 * min(
            pyq.Quaternion.distance(quat_1, quat_2),
            pyq.Quaternion.distance(quat_1, -quat_2)
        )

        if d > self.pos_tolerance:
            cause += f"Position error ({d} m) is greater than the tolerance ({self.pos_tolerance} m). "
        if phi > self.orient_tolerance:
            cause += f"Orientation error ({phi} rad) is greater than the tolerance ({self.orient_tolerance} rad). "

        if cause == '':
            cause = 'Pose termination handler caused by: goal pose reached.'
            terminate = True
        
        termination_signal = TerminationSignal()
        termination_signal.id = self.id
        termination_signal.terminate = terminate
        termination_signal.cause = cause
        return termination_signal