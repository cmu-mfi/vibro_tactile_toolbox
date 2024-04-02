#!/usr/bin/env python3

import rospy
import json
from vibro_tactile_toolbox.msg import SkillParams, TerminationSignal, TerminationConfig

from robot_controller.robot_commander import BaseRobotCommander

from typing import Tuple, List

class BaseSkill:

    def __init__(self, robot_commander: BaseRobotCommander):

        self.robot_commander = robot_commander

        self.skill_termination_sub = rospy.Subscriber("/terminator/skill_termination_signal", TerminationSignal, self.skill_termination_callback)
        self.termination_config_pub = rospy.Publisher("/terminator/termination_config", TerminationConfig, queue_size=1)
        self.skill_param_pub = rospy.Publisher("/skill/param", SkillParams, queue_size=1)

        self.outcome_srv = None

        self.skill_steps = []

    def execute_skill(self, params) -> Tuple[List[TerminationSignal], List[int]]:
        """
        Function called by task script to execute all steps of skill and return
        """
        terminals = []
        outcomes = []

        for skill_step, param in zip(self.skill_steps, params):
            print(f"Executing skill step: \'{skill_step['step_name']}\'")
            skill_id = rospy.Time.now().secs
            termination_cfg = skill_step['termination_cfg']
            termination_cfg['id'] = skill_id
            if 'termination' in param.keys():
                termination_cfg.update(param['termination'])
            skill_param_msg = SkillParams()
            skill_param_msg.id = skill_id
            skill_param_msg.skill_name = self.__class__.__name__
            skill_param_msg.step_name = skill_step['step_name']
            skill_param_msg.param = json.dumps(param)
            self.skill_param_pub.publish(skill_param_msg)

            terminal, outcome = self.execute_skill_step(skill_step['command'], param['skill'], termination_cfg)
            terminals.append(terminal)
            outcomes.append(outcome)

        return terminals, outcomes

    def execute_skill_step(self, robot_command, skill_param, termination_config) -> Tuple[TerminationSignal, int]:
        """
        Execute a single skill step
        robot_command: Lambda function of a populated XXRobotCommander method
        termination_config: TerminationConfig for this step
        """
        # 1. Set the termination config and reset signal
        termination_cfg_msg = TerminationConfig()
        termination_cfg_msg.cfg_json = json.dumps(termination_config)
        self.termination_config_pub.publish(termination_cfg_msg)
        self.termination_signal = None

        rospy.sleep(1)

        # 2. Send the robot command
        robot_command(skill_param)

        # 3. Wait for termination
        termination_signal = rospy.wait_for_message("/terminator/skill_termination_signal", TerminationSignal, rospy.Duration(20))

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
