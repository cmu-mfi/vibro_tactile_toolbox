#!/usr/bin/env python3
from __future__ import print_function

import sys
import math
import numpy as np
from typing import List, Optional, Union
import pyquaternion as pyq

import rospy
import actionlib
import std_msgs.msg

from geometry_msgs.msg import Pose, PoseStamped, Twist
from sensor_msgs.msg import JointState

from yk_msgs.srv import *
from yk_msgs.msg import *
from std_srvs.srv import Trigger, TriggerRequest

from robot_controller.base_robot_commander import BaseRobotCommander

def pose_to_list(pose: Pose):
    return [pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

def _joints_close(goal: List, actual: List, tolerance: float):
    """
    Check if the joint values in two lists are within a tolerance of each other
    """
    for index in range(len(goal)):
        if abs(actual[index] - goal[index]) > tolerance:
            err = goal[index] - actual[index]
            rospy.logwarn(
                f"Joint error ({err} rad) is greater than the tolerance ({tolerance} rad)"
            )
            return False
    return True

def _poses_close(
    goal: Union[Pose, PoseStamped],
    actual: Union[Pose, PoseStamped],
    pos_tolerance: float,
    orient_tolerance: float,
):
    """
    Check if the actual and goal poses are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    """

    goal = goal.pose if isinstance(goal, PoseStamped) else goal
    actual = actual.pose if isinstance(actual, PoseStamped) else actual

    x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
    x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
    # Euclidean distance
    d = math.dist((x1, y1, z1), (x0, y0, z0))
    # angle between orientations
    quat_1 = pyq.Quaternion(qx0, qy0, qz0, qw0)
    quat_2 = pyq.Quaternion(qx1, qy1, qz1, qw1)
    phi = 2 * min(
        pyq.Quaternion.distance(quat_1, quat_2),
        pyq.Quaternion.distance(quat_1, -quat_2)
    )

    result = True
    if d > pos_tolerance:
        rospy.logwarn(
            f"Position error ({d} m) is greater than the tolerance ({pos_tolerance} m)"
        )
        result = False
    if phi > orient_tolerance:
        rospy.logwarn(
            f"Orientation error ({phi} rad) is greater than the tolerance ({orient_tolerance} rad)"
        )
        result = False
    return result

class YaskawaRobotController(BaseRobotCommander):

    def __init__(self, namespace: str = '/', verbose: bool = False) -> None:

        self.joints = [
            "joint_1_s",
            "joint_2_l",
            "joint_3_u",
            "joint_4_r",
            "joint_5_b",
            "joint_6_t",
        ]

        self.namespace = namespace
        self.robot_enable_client = rospy.ServiceProxy(f'/{namespace}/robot_enable', Trigger)
        self.get_pose_client = rospy.ServiceProxy(f'/{namespace}/yk_get_pose', GetPose)
        self.get_pose_stamped_client = rospy.ServiceProxy(f'/{namespace}/yk_get_pose_stamped', GetPoseStamped)
        self.go_to_pose_client = actionlib.SimpleActionClient(f'/{namespace}/yk_go_to_pose', GoToPoseAction)
        self.go_to_joints_client = actionlib.SimpleActionClient(f'/{namespace}/yk_go_to_joints', GoToJointsAction)
        self.stop_srv_client = rospy.ServiceProxy(f'/{namespace}/yk_stop_trajectory', Trigger)
        #self.execute_joint_trajectory_client = actionlib.SimpleActionClient(f'/{namespace}/yk_execute_trajectory', ExecuteCartesianTrajectoryAction)

        rospy.wait_for_service(f'/{namespace}/yk_get_pose')
        rospy.loginfo("Found get pose server.")
        rospy.wait_for_service(f'/{namespace}/yk_get_pose_stamped')
        rospy.loginfo("Found get pose stamped server.")
        self.go_to_pose_client.wait_for_server()
        rospy.loginfo("Found go to pose server.")
        self.go_to_joints_client.wait_for_server()
        rospy.loginfo("Found go to joints server.")
        rospy.wait_for_service(f'/{namespace}/yk_stop_trajectory')
        rospy.loginfo("Found stop server.")
        #self.execute_joint_trajectory_client.wait_for_server()
        rospy.loginfo("Found execute joint trajectory client")

        self.joint_state_topic = f'/{namespace}/joint_state'
        self.HOME_JOINTS = [0, 0, 0, 0, -np.pi/2, 0]

        self.current_skill = None


    def enable_robot(self):
        # Direct to yk
        trigger_req = TriggerRequest()
        self.robot_enable_client(trigger_req)

    def stop(self):
        # Direct to yk
        trigger_req = TriggerRequest()
        self.stop_srv_client(trigger_req)

    def wait_for_skill(self):
        if current_skill is not None:
            if current_skill == "joint":
                resp = self.go_to_joints_client.wait_for_result()
                current_skill = None
                return True
            elif current_skill == "pose":
                self.go_to_pose_client.wait_for_result()
                current_skill = None
                return True
        return False

    def go_home(self, 
        cartesian_path: bool = False,
        tolerance: float = 0.001,
        velocity_scaling: float = 0.01,
        acc_scaling: float = 0.01,
        wait: bool = True):
        return self.go_to_joints(self.HOME_JOINTS, cartesian_path, tolerance, velocity_scaling, acc_scaling, wait)


    def get_current_pose(self, end_effector_link: str = "base_link", stamped: bool = False) -> Union[Pose, PoseStamped]:
        """
        position in metres. orientation in radians
        """
        if stamped:
            return self.get_pose_stamped_client(end_effector_link).pose
        else:
            return self.get_pose_client(end_effector_link).pose

    def get_current_joints(self, in_degrees: bool = False) -> List:
        """
        joint angle values in radians (or) degrees
        """
        joint_state = rospy.wait_for_message(self.joint_state_topic, JointState, timeout=1)
        if in_degrees:
            joint_state = [np.rad2deg(joint) for joint in joint_state]
        return joint_state

    def go_to_joints(
        self,
        joint_goal: List[float],
        cartesian_path: bool = False,
        tolerance: float = 0.001,
        velocity_scaling: float = 0.05,
        acc_scaling: float = 0.05,
        wait: bool = True,
    ) -> bool:
        self.enable_robot()
        goal = GoToJointsGoal()
        goal.state.name = self.joints
        goal.state.position = joint_goal
        goal.max_velocity_scaling_factor = velocity_scaling
        goal.max_acceleration_scaling_factor = acc_scaling
        self.go_to_joints_client.wait_for_server()
        self.go_to_joints_client.send_goal(goal)
        current_skill = "joint"
        if wait:
            resp = self.go_to_joints_client.wait_for_result()
            current_skill = None
            return _joints_close(joint_goal, self.go_to_joints_client.get_result().state.position, tolerance)
        return True

    def go_to_pose_goal(
        self,
        pose_goal: Pose,
        cartesian_path=True,
        pos_tolerance: float = 0.001,
        orient_tolerance: float = 0.01,
        velocity_scaling: float = 0.1,
        eef_frame: Optional[str] = "base_link",
        acc_scaling: float = 0.1,
        wait: bool = True,
    ) -> bool:
        self.enable_robot()
        goal = GoToPoseGoal()
        goal.pose = pose_goal
        goal.base_frame = eef_frame
        goal.max_velocity_scaling_factor = velocity_scaling
        goal.max_acceleration_scaling_factor = acc_scaling
        self.go_to_pose_client.wait_for_server()
        self.go_to_pose_client.send_goal(goal)
        current_skill = "pose"
        if wait:
            self.go_to_pose_client.wait_for_result()
            current_skill = None
            return _poses_close(
                pose_goal, self.get_current_pose(eef_frame), pos_tolerance, orient_tolerance
            )
        return True


    def send_cartesian_pos_trajectory(
        self, pose_list: List[Pose], wait: bool = False
    ):
        """
        Sends a cartesian position trajectory to the robot
        """

        raise NotImplementedError
