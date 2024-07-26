#!/usr/bin/env python3

import abc

class BaseGripperController(object, metaclass=abc.ABCMeta):
    """
    Abstract base class for GripperController objects
    """
    def stop(self):
        """
        Stop the gripper from moving
        """
        raise NotImplementedError

    def wait_for_gripper(self):
        """
        Wait for the gripper to finish moving
        """
        raise NotImplementedError

    def close(self, 
        velocity: float = 0.15, # m/s
        force: float = 20,      # N
        wait: bool = True) -> bool:
        """
        Close the gripper all the way
        """
        raise NotImplementedError

    def open(self, 
        velocity: float = 0.15, # m/s
        force: float = 20,      # N 
        wait: bool = True) -> bool:
        """
        Open the gripper all the way
        """
        raise NotImplementedError


    def go_to_width(
        self,
        position: float,        # m
        velocity: float = 0.15, # m/s
        force: float = 20,      # N 
        wait: bool = True) -> bool:
        """
        Move the gripper to the desired position
        """
        raise NotImplementedError

    def get_width(self) -> float:
        """
        Get the current gripper width
        """
        raise NotImplementedError