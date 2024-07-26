#!/usr/bin/env python3
from __future__ import print_function

import sys
import math
import numpy as np

import rospy
import actionlib

from robotiq_mm_ros.srv import *
from robotiq_mm_ros.msg import *

from gripper_controller.base_gripper_controller import BaseGripperController

class RobotiqHandEController(BaseGripperController):

    def __init__(self, namespace: str = '/', verbose: bool = False) -> None:

        self.namespace = namespace
        self.min_width = 0.0
        self.max_width = 0.05
        self.min_velocity = 0.02
        self.max_velocity = 0.15
        self.min_force = 20
        self.max_force = 185

        self.get_gripper_pos_client = rospy.ServiceProxy(f'/{namespace}/get_gripper_pos', GetGripperPos)
        self.move_gripper_client = actionlib.SimpleActionClient(f'/{namespace}/move_gripper', GripperCommandAction)


    def stop(self):
        self.move_gripper_client.cancel_goal()

    def wait_for_gripper(self):
        self.move_gripper_client.wait_for_result()
        return self.move_gripper_client.get_result()

    def close(self, 
        velocity: float = 0.15, # m/s
        force: float = 20,      # N
        wait: bool = True) -> bool:
        
        return self.go_to_width(self.min_width, velocity, force, wait)

    def open(self, 
        velocity: float = 0.15, # m/s
        force: float = 20,    # N 
        wait: bool = True):
        
        return self.go_to_width(self.max_width, velocity, force, wait)


    def go_to_width(self,
        position: float,
        velocity: float = 0.15,
        force: float = 20,
        wait: bool = True) -> bool:
        """
        Move the gripper to the desired position
        """

        self.move_gripper_client.wait_for_server()

        goal = GripperCommandGoal(position=position, velocity=velocity, force=force)

        # Sends the goal to the action server.
        self.move_gripper_client.send_goal(goal)

        if wait:
            return self.wait_for_gripper()
        else:
            return True

    def get_width(self) -> float:
        """
        Get the current gripper width
        """
        return self.get_gripper_pos_client()