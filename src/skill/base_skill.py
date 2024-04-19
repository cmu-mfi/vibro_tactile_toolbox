#!/usr/bin/env python3

import rospy
from rospy import Publisher
import json
from vibro_tactile_toolbox.msg import SkillParams, TerminationSignal, TerminationConfig
from std_msgs.msg import String

from robot_controller.base_robot_commander import BaseRobotCommander

from typing import Tuple, List

def create_skill_publishers(namespace):
    publishers = {}
    publishers['skill_param_pub'] = rospy.Publisher(f'/{namespace}/skill/param', SkillParams, queue_size=10)
    publishers['termination_config_pub'] = rospy.Publisher(f'/{namespace}/terminator/termination_config', TerminationConfig, queue_size=10)
    while publishers['termination_config_pub'].get_num_connections() == 0:
        rospy.loginfo("Waiting for subscriber to connect")
        rospy.sleep(1)
    return publishers

class BaseSkill:
    NULL_PARAM = {'skill': None,
                  'termination': None,
                  'outcome': None}
    
    def __init__(self, robot_commander: BaseRobotCommander, namespace: str):

        self.namespace = namespace

        self.robot_commander = robot_commander
        self.skill_termination_topic_name = f'/{namespace}/terminator/skill_termination_signal'

        publishers = create_skill_publishers(namespace)

        self.skill_param_pub = publishers['skill_param_pub']
        self.termination_config_pub = publishers['termination_config_pub']
        
        self.outcome_srv = None

        self.skill_steps = []

    def execute_skill(self, params) -> Tuple[List[TerminationSignal], List[int]]:
        """
        Function called by task script to execute all steps of skill and return
        """
        # Load execution params
        skill_step_delay = None
        max_retry_attempts = None
        skill_step_params = None
        if params:
            # delay between skill steps in seconds (float)
            if 'skill_step_delay' in params:
                skill_step_delay = params['skill_step_delay']
            # how many times a skill step can be re-attempted
            if 'max_retry_attempts' in params:
                max_retry_attempts = params['max_retry_attempts']
            # pre-loaded parameters for listed skill step(s) (NULL_PARAM like)
            if 'skill_step_params' in params:
                skill_step_params = params['skill_step_params']

        terminals = []
        outcomes = []
        for skill_step in self.skill_steps:
            print(f"Executing skill step: \'{skill_step['step_name']}\'")

            # Read in any pre-loaded step parameter
            if skill_step_params and skill_step['step_name'] in skill_step_params:
                step_param = skill_step_params[skill_step['step_name']]
            else:
                step_param = BaseSkill.NULL_PARAM

            # Execute the skill step and record the termination cause and outcome
            terminal, outcome = self.execute_skill_step(skill_step, step_param)
            terminals.append(terminal)
            outcomes.append(outcome)

            # TODO: If outcome is unsuccessful:
            #   Query some policy to get a new step_param
            #   Retry execute_skill_step with new step_param
            #   Loop for max_retry_attempts
            #   If max reached then abort?

            # Delay between steps to make data segmentation easier
            if skill_step_delay:
                rospy.sleep(skill_step_delay)

        return terminals, outcomes

    def execute_skill_step(self, skill_step, param) -> Tuple[TerminationSignal, int]:
        """
        Execute a single skill step
        robot_command: Lambda function of a populated XXRobotCommander method
        termination_config: TerminationConfig for this step
        """

        #1 Do some calculations

        if 'calculate' in skill_step and 'calculate' in param:
            skill_step['calculate'](param['calculate'])

        skill_id = rospy.Time.now().secs
        skill_param_msg = SkillParams()
        skill_param_msg.id = skill_id
        skill_param_msg.skill_name = self.__class__.__name__
        skill_param_msg.step_name = skill_step['step_name']
        skill_param_msg.param = json.dumps(param)
        self.skill_param_pub.publish(skill_param_msg)


        # 1. Set the termination config and reset signal
        termination_cfg = skill_step['termination_cfg'](param['termination'])
        termination_cfg['id'] = skill_id
        termination_cfg_msg = TerminationConfig()
        termination_cfg_msg.cfg_json = json.dumps(termination_cfg)
        self.termination_config_pub.publish(termination_cfg_msg)

        # 2. Send the robot command
        skill_step['robot_command'](param['skill'])

        # 3. Wait for terminationgit
        termination_signal = rospy.wait_for_message(self.skill_termination_topic_name, TerminationSignal, rospy.Duration(20))

        # 3.1 Stop terminators (this prevents the terminator node from filling with wasted messages)
        termination_cfg = {}
        termination_cfg['id'] = rospy.Time.now().secs
        termination_cfg_msg = TerminationConfig()
        termination_cfg_msg.cfg_json = json.dumps(termination_cfg)
        self.termination_config_pub.publish(termination_cfg_msg)

        # 4. Get outcome
        if 'outcome' in skill_step:
            outcome = skill_step['outcome'](param['outcome'])
        else:
            outcome = None

        # 5. Return terminal and outcome
        return termination_signal, outcome


    def stop_robot(self):
        # Call robot stop service
        try:
            self.stop_service()
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call stop service: %s" % e)
