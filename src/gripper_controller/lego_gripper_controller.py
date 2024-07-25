#!/usr/bin/env python3
from __future__ import print_function

import sys
import math
import numpy as np

import rospy

from gripper_controller.base_gripper_controller import BaseGripperController

class LegoGripperController(BaseGripperController):

    def __init__(self, namespace: str = '/', verbose: bool = False) -> None:
        pass

    def stop(self):
        return True

    def wait_for_gripper(self):
        return True

    def close(self, 
        velocity: float = 0.15, # m/s
        force: float = 20,      # N
        wait: bool = True) -> bool:
        
        return True

    def open(self, 
        velocity: float = 0.15, # m/s
        force: float = 20,    # N 
        wait: bool = True):
        
        return True


    def go_to_width(self,
        position: float,
        velocity: float = 0.15,
        force: float = 20,
        wait: bool = True) -> bool:
        """
        Move the gripper to the desired position
        """

        return True

    def get_width(self) -> float:
        """
        Get the current gripper width
        """
        return 0.0