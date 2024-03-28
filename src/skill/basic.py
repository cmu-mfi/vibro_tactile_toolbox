#!/usr/bin/env python3

import rospy
from vibro_tactile_toolbox.msg import TerminationSignal, TerminationConfig

from robot_controller.robot_commander import BaseRobotCommander
import terminator.utils as t_utils

from typing import Tuple, List

class Basic:
    robot_commander: BaseRobotCommander

    def __init__(self, robot_commander: BaseRobotCommander):
        rospy.init_node('basic_skill_execution_node')

        self.robot_commander = robot_commander

        self.skill_termination_sub = rospy.Subscriber("terminator/skill_termination_signal", TerminationSignal, self.skill_termination_callback)
        self.termination_config_pub = rospy.Publisher("terminator/termination_config", TerminationConfig)

        self.outcome_srv = None

        self.default_skill_steps = [
             {'step_name': 'home',
              'command': lambda: self.robot_commander.go_home(),
              'termination_cfg': t_utils.make_termination_cfg(id=1, timeout={'duration': 10})}
        ]

    def execute_skill(self) -> Tuple[List[TerminationSignal], List[int]]:
        """
        Funciton called by task script to execut all steps of skill and return
        """
        terminals = []
        outcomes = []
        for skill_step in self.default_skill_steps:
            print(f"Executing skill step: \'{skill_step['step_name']}\'")
            terminal, outcome = self.execute_skill_step(skill_step['command'], skill_step['termination_cfg'])
            terminals.append(terminal)
            outcomes.append(outcome)

        # Shut down node
        # rospy.signal_shutdown()
        return terminals, outcomes

    def execute_skill_step(self, robot_command, termination_config) -> Tuple[TerminationSignal, int]:
        """
        Execute a single skill step
        robot_command: Lambda function of a populated XXRobotCommander method
        termination_config: TerminationConfig for this step
        """
        # 1. Set the termination config and reset signal
        self.termination_config_pub.publish(termination_config)
        self.termination_signal = None

        # 2. Send the robot command
        robot_command()

        # 3. Wait for termination
        termination_signal = rospy.wait_for_message("terminator/skill_termination_signal", TerminationSignal, rospy.Duration(20))

        # 4. Get outcome
        outcome = 0

        # 5. Return terminal and outcome
        return termination_signal, outcome


    def skill_termination_callback(self, msg):
        # Check termination conditions
        # Perform termination actions if necessary
        if msg.terminate:
            rospy.loginfo("Skill termination signal received. Stopping the robot.")
            self.robot_commander.stop()
            self.termination_signal = msg

    def stop_robot(self):
        # Call robot stop service
        try:
            self.stop_service()
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call stop service: %s" % e)
