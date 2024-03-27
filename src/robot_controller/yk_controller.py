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

from yk_tasks.srv import *
from std_srvs.srv import Trigger, TriggerRequest


class YaskawaRobotController(BaseRobotCommander):

    def __init__(self, namespace: str = '/', verbose: bool = False) -> None:

        self.namespace = f'/{namespace}'
        self.get_pose_client = rospy.ServiceProxy(f'/{namespace}/yk_get_pose', GetPose)
        self.get_pose_stamped_client = rospy.ServiceProxy(f'/{namespace}/yk_get_pose_stamped', GetPoseStamped)
        self.set_pose_client = rospy.ServiceProxy(f'/{namespace}/yk_set_pose', SetPose)
        self.set_pose_client = rospy.ServiceProxy(f'/{namespace}/yk_set_joints', SetJoints)
        self.stop_srv_client = rospy.ServiceProxy(f'/{namespace}/robot_disable', Trigger)

        self.joint_state_topic = f'/{namespace}/joint_state'
        self.HOME_JOINTS = [0, 0, 0, 0, -np.pi/2, 0]


    def stop(self):
        # Direct to yk
        trigger_req = TriggerRequest()
        self.srv_client(trigger_req)

        # Movegroup stop
        # self.move_group.stop()

    def go_home(self, 
        cartesian_path: bool = False,
        tolerance: float = 0.001,
        velocity_scaling: float = 0.01,
        acc_scaling: float = 0.01,
        wait: bool = True):
        return self.go_to_joints(self.HOME_JOINTS, cartesian_path, tolerance, velocity_scaling, acc_scaling, wait)


    def get_current_pose(self, end_effector_link: str = "", stamped: bool = False) -> Union[Pose, PoseStamped]:
        """
        position in metres. orientation in radians
        """
        if stamped:
            return self.get_pose_stamped_client(end_effector_link)
        else:
            return self.get_pose_client(end_effector_link)

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
        # Check if MoveIt planner is running
        #rospy.wait_for_service("/plan_kinematic_path", self.timeout)
        # Create a motion planning request with all necessary goals and constraints
        mp_req = mi_msg.MotionPlanRequest()
        mp_req.pipeline_id = "pilz_industrial_motion_planner"
        mp_req.planner_id = "LIN" if cartesian_path else "PTP"
        mp_req.group_name = "manipulator"
        mp_req.num_planning_attempts = 1

        constraints = mi_msg.Constraints()
        for joint_no in range(len(self.JOINTS)):
            constraints.joint_constraints.append(mi_msg.JointConstraint())
            constraints.joint_constraints[-1].joint_name = self.JOINTS[joint_no]
            constraints.joint_constraints[-1].position = joint_goal[joint_no]
            constraints.joint_constraints[-1].tolerance_above = tolerance
            constraints.joint_constraints[-1].tolerance_below = tolerance

        mp_req.goal_constraints.append(constraints)
        mp_req.max_velocity_scaling_factor = velocity_scaling
        mp_req.max_acceleration_scaling_factor = acc_scaling

        mp_res = self.get_plan(mp_req).motion_plan_response
        if mp_res.error_code.val != mp_res.error_code.SUCCESS:
            rospy.logerr(
                "Planner failed to generate a valid plan to the goal joint_state"
            )
            return False
        if (
            len(mp_res.trajectory.joint_trajectory.points) > 1
            and mp_res.trajectory.joint_trajectory.points[-1].time_from_start
            == mp_res.trajectory.joint_trajectory.points[-2].time_from_start
        ):
            mp_res.trajectory.joint_trajectory.points.pop(-2)
            rospy.logwarn(
                "Duplicate time stamp in the planned trajectory. Second last way-point was removed."
            )
        goal = mi_msg.ExecuteTrajectoryGoal(trajectory=mp_res.trajectory)
        self.execute_plan.wait_for_server()
        self.execute_plan.send_goal(goal)
        if wait:
            self.execute_plan.wait_for_result()
            # Calling ``stop()`` ensures that there is no residual movement
            self.move_group.stop()
            return _joints_close(joint_goal, self.get_current_joints(), tolerance)
        return True

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

        if eef_frame is None:
            eef_frame = self.eef_frame
        # Check if MoveIt planner is running
        #rospy.wait_for_service("/plan_kinematic_path", self.timeout)
        # Create a motion planning request with all necessary goals and constraints
        mp_req = mi_msg.MotionPlanRequest()
        mp_req.pipeline_id = "pilz_industrial_motion_planner"
        mp_req.planner_id = "LIN" if cartesian_path else "PTP"
        mp_req.group_name = "manipulator"
        mp_req.num_planning_attempts = 5

        mp_req_pose_goal = PoseStamped(
            header=std_msgs.msg.Header(frame_id=self.planning_frame), pose=pose_goal
        )

        constraints = constructGoalConstraints(
            eef_frame, mp_req_pose_goal, pos_tolerance, orient_tolerance
        )
        mp_req.goal_constraints.append(constraints)
        mp_req.max_velocity_scaling_factor = velocity_scaling
        mp_req.max_acceleration_scaling_factor = acc_scaling

        mp_res = self.get_plan(mp_req).motion_plan_response
        if mp_res.error_code.val != mp_res.error_code.SUCCESS:
            rospy.logerr("Planner failed to generate a valid plan to the goal pose")
            return False
        goal = mi_msg.ExecuteTrajectoryGoal(trajectory=mp_res.trajectory)
        self.execute_plan.wait_for_server()
        self.execute_plan.send_goal(goal)
        if wait:
            self.execute_plan.wait_for_result()
            # Calling ``stop()`` ensures that there is no residual movement
            self.move_group.stop()
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
