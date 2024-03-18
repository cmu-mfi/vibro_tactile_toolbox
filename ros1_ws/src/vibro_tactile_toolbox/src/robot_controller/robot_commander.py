#!/usr/bin/env python3

import abc

from typing import List, Optional, Union

from geometry_msgs.msg import Pose, PoseStamped

class BaseRobotCommander(object, metaclass=abc.ABCMeta):
    """
    Abstract base class for RobotCommander objects
    """
    def stop(self):

        raise NotImplementedError

    def go_home(self, 
        cartesian_path: bool = False,
        tolerance: float = 0.001,
        velocity_scaling: float = 0.01,
        acc_scaling: float = 0.01,
        wait: bool = True) -> bool:
        
        raise NotImplementedError


    def get_current_pose(self, end_effector_link: str = "", stamped: bool = False) -> Union[Pose, PoseStamped]:
        """
        Get the current EE pose. position in metres. orientation in radians
        """
        raise NotImplementedError

    def get_current_joints(self, in_degrees: bool = False) -> List:
        """
        Get the current joint angles. joint angle values in radians (default) or degrees if in_degrees is True.
        """
        raise NotImplementedError

    def go_to_joints(
        self,
        joint_goal: List[float],
        cartesian_path: bool = False,
        tolerance: float = 0.001,
        velocity_scaling: float = 0.05,
        acc_scaling: float = 0.05,
        wait: bool = True,
    ) -> bool:
        """
        Move the robot to the joint_goal state.
        """
        raise NotImplementedError

    def go_to_pose_goal(
        self,
        pose_goal: Pose,
        cartesian_path=True,
        pos_tolerance: float = 0.001,
        orient_tolerance: float = 0.01,
        velocity_scaling: float = 0.1,
        eef_frame: Optional[str] = None,
        acc_scaling: float = 0.1,
        wait: bool = True,
    ) -> bool:
        """
        Move the robot EE to the pose_goal.
        """
        raise NotImplementedError

    def send_cartesian_pos_trajectory(
        self, pose_list: List[Pose], wait: bool = False
    ) -> bool:
        """
        Move the robot through a cartesian trajectory specified by pose_list.
        """
        raise NotImplementedError