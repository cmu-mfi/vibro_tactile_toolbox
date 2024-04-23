#!/usr/bin/env python3

import copy
import numpy as np
import subprocess
import signal
import os
import rospy
import json
from geometry_msgs.msg import Pose, Twist,Wrench
from autolab_core import RigidTransform

from robot_controller.base_robot_commander import BaseRobotCommander
from skill.base_skill import BaseSkill
import terminator.utils as t_utils

from vibro_tactile_toolbox.msg import TerminationSignal
from vibro_tactile_toolbox.srv import *
from vibro_tactile_toolbox.msg import *

from typing import Tuple, List

### Lego specific globals ###

STUD_WIDTH = 0.008 # 8 mm
LEGO_BLOCK_HEIGHT=0.0096 # z 9.6mm
LEGO_BLOCK_WIDTH=1*STUD_WIDTH #x    #0.0158 (with clearance)
LEGO_BLOCK_LENGTH=2*STUD_WIDTH #y   #0.0318 (with clearance)

### Termination Configs for Lego Skills ###
# Rapid motion
rapid_termination_config = {
    'time': {'duration': 10.0},
    'pose': {'pos_tolerance': 0.001,
                'orient_tolerance': 0.01,
                'pose': None},
}
# Engaging a brick on the build plate
engage_termination_config = {
    'time': {'duration': 10.0},
    'pose': {'pos_tolerance': 0.001,
                'orient_tolerance': 0.01,
                'pose': None},
    'fts': {'check_rate_ns': 1E6,
            'threshold': {
                'force': {
                    'x': 10,
                    'y': 10,
                    'z': 20},
                'torque': {
                    'x': 1.0,
                    'y': 1.0,
                    'z': 1.0}
            }}
}
# Releasing a brick through rotation
release_termination_config = {
    'time': {'duration': 10.0},
    'pose': {'pos_tolerance': 0.001,
                'orient_tolerance': 0.01,
                'pose': None},
    'fts': {'check_rate_ns': 1E6,
            'threshold': {
                'force': {
                    'x': 10,
                    'y': 10,
                    'z': 20},
                'torque': {
                    'x': 1.0,
                    'y': 1.0,
                    'z': 1.0}
            }}
}
# Pulling up on a brick to check if connected
servo_termination_config = {
    'time': {'duration': 10.0},
    'pose': {'pos_tolerance': 0.001,
                'orient_tolerance': 0.01,
                'pose': None},
    'fts': {'check_rate_ns': 1E6,
            'threshold': {
                'force': {
                    'x': 10,
                    'y': 10,
                    'z': 20},
                'torque': {
                    'x': 1.0,
                    'y': 1.0,
                    'z': 1.0}
            }}
}

def add_termination_pose(termination_config, pose : Pose):
    t_cfg = copy.deepcopy(termination_config)
    t_cfg['pose']['pose'] = t_utils.pose_to_dict(pose)
    return t_cfg

class PlaceLegoHardcodedCorrectionSkill(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, namespace: str):

        super().__init__(robot_commander, namespace)

        self.skill_steps = [
            {'step_name': 'go_to_approach_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.approach_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.approach_pose_msg),
             'outcome': lambda param: self.send_start_outcome_request(param)},
            {'step_name': 'engage_brick_bad',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.place_pose_bad_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(engage_termination_config, self.place_pose_bad_msg)},
            {'step_name': 'reset_to_approach_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.approach_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.approach_pose_msg),
             'outcome': lambda param: self.send_start_outcome_request(param)},
            {'step_name': 'engage_brick_good',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.place_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(engage_termination_config, self.place_pose_msg)},
            {'step_name': 'pull_up',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.servo_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(servo_termination_config, self.servo_pose_msg),
             'outcome': lambda param: self.send_end_fts_outcome_request(param)},
            {'step_name': 'reengage_brick',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.place_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(engage_termination_config, self.place_pose_msg)},
            {'step_name': 'release_brick',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.release_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(release_termination_config, self.release_pose_msg),
             'outcome': lambda param: self.send_end_vision_outcome_request(param)},
            {'step_name': 'disengage_brick',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.disengage_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.disengage_pose_msg)}
        ]

    def execute_skill(self, execution_params, skill_params) -> Tuple[List[TerminationSignal], List[int]]:
        # QOL 
        if 'T_lego_ee' not in skill_params:
            print(f"PlaceLegoSkill expects end effector transform: skill_params['T_lego_ee'] = RigidTransform()")
        if 'T_lego_world' not in skill_params:
            print(f"PlaceLegoSkill expects target brick pose transform: skill_params['T_lego_world'] = RigidTransform()")
        if 'approach_height_offset' not in skill_params:
            print(f"PlaceLegoSkill expects an approach height offset (meters): skill_params['approach_height_offset'] = float(0.010 m)")
        if 'place_rotation' not in skill_params:
            print(f"PlaceLegoSkill expects a release rotation (deg): skill_params['place_rotation'] = float(30 deg)")
        if 'place_perturbation' not in skill_params:
            self.place_perturbation = np.random.uniform(0.000, 0.001, size=(3,))
            print(f"No initial place perturbation specified, using random perturbation: {tuple(self.place_perturbation)} [m]")
        else:
            self.place_perturbation = np.array(skill_params['place_perturbation'])

        self.T_lego_ee = skill_params['T_lego_ee']
        self.T_lego_world = skill_params['T_lego_world']
        self.approach_height_offset = skill_params['approach_height_offset']
        self.place_rotation = skill_params['place_rotation']

        self.T_lego_approach = RigidTransform(
            translation=np.array([0.0, 0.0, -abs(self.approach_height_offset)]),
            from_frame='lego', to_frame='lego'
        )
        self.T_lego_rotation = RigidTransform(
            rotation=RigidTransform.y_axis_rotation(np.deg2rad(self.place_rotation)), 
            translation=np.array([LEGO_BLOCK_WIDTH/2,0,0]), 
            from_frame='lego', to_frame='lego'
        )
        self.T_lego_rotation_point_offset = RigidTransform(
            translation=np.array([LEGO_BLOCK_WIDTH/2,0,0]), 
            from_frame='lego', to_frame='lego'
        )
        self.T_lego_servo = RigidTransform(
            translation=np.array([0.0, 0.0, -0.001]),
            from_frame='lego', to_frame='lego'
        )
        self.T_lego_disengage = RigidTransform(

        )

        self.T_lego_place_bad = RigidTransform(translation=self.place_perturbation,
                                               from_frame='lego', to_frame='lego')

        self.T_place_target = self.T_lego_world.copy()
        
        self.place_pose = self.T_place_target * self.T_lego_ee.inverse()
        self.place_pose_msg = self.place_pose.pose_msg

        self.place_pose_bad = self.T_place_target * self.T_lego_place_bad * self.T_lego_ee.inverse()
        self.place_pose_bad_msg = self.place_pose_bad.pose_msg

        self.approach_pose = self.T_place_target * self.T_lego_approach * self.T_lego_ee.inverse()
        self.approach_pose_msg = self.approach_pose.pose_msg

        self.release_pose = self.T_place_target * self.T_lego_rotation * self.T_lego_rotation_point_offset.inverse() * self.T_lego_ee.inverse()
        self.release_pose_msg = self.release_pose.pose_msg

        self.servo_pose = self.T_place_target * self.T_lego_servo * self.T_lego_ee.inverse()
        self.servo_pose_msg = self.servo_pose.pose_msg

        self.disengage_pose = self.T_place_target * self.T_lego_approach
        self.disengage_pose *= self.T_lego_rotation * self.T_lego_rotation_point_offset.inverse() * self.T_lego_ee.inverse()
        self.disengage_pose_msg = self.disengage_pose.pose_msg

        return super().execute_skill(execution_params)

    def send_start_outcome_request(self, param):
        rospy.wait_for_service('/fts_detector')
        rospy.wait_for_service('/lego_detector')

        try:
            
            detect_fts = rospy.ServiceProxy('/fts_detector', FTSOutcome)
            fts_req = FTSOutcomeRequest()
            fts_req.id = 0
            fts_req.topic_name = 'fts'  
            fts_req.start = True
            fts_req.threshold = Wrench()
            fts_req.threshold.force.x = 10
            fts_req.threshold.force.y = 10
            fts_req.threshold.force.z = 2.0
            fts_req.threshold.torque.x = 10
            fts_req.threshold.torque.y = 10
            fts_req.threshold.torque.z = 10
            

            fts_resp = detect_fts(fts_req)
            fts_result = json.loads(fts_resp.result)
            print("FTS Detector Response:", fts_resp.result)
            

            detect_lego = rospy.ServiceProxy('/lego_detector', LegoOutcome)
            lego_req = LegoOutcomeRequest()
            lego_req.id = 0
            lego_req.topic_name = '/side_camera/color/image_cropped'
            lego_req.start = True
            lego_req.score_threshold = 0.8
            
            top_bbox = BoundingBox(coords=[800, 200, 1100, 350])
            bot_bbox = BoundingBox(coords=[800, 350, 1100, 500])
            lego_req.top_bbox = top_bbox
            lego_req.bot_bbox = bot_bbox

            lego_resp = detect_lego(lego_req)
            lego_result = json.loads(lego_resp.result)
            print("Vision Detection Response:", lego_resp.result)
            return lego_result["starting_top"] ==0 and lego_result["starting_bottom"] ==0 and fts_result["starting_forces"][2]<fts_req.threshold.force.z
    

        except rospy.ServiceException as e:
            print("Service call failed:", e)
            return None


    
    # def send_end_fts_outcome_request(self, param):
    #     return -1
    #     raise NotImplementedError
    
    def send_end_fts_outcome_request(self, param):
        rospy.wait_for_service('/fts_detector')
        try:
          
            detect_fts = rospy.ServiceProxy('/fts_detector', FTSOutcome)
            
            
            req = FTSOutcomeRequest()
            req.id = 0  
            req.topic_name ='fts'
            req.start = False  
            req.threshold = Wrench()
            
       
            req.threshold.force.x = 10
            req.threshold.force.y = 10
            req.threshold.force.z =2.0
            req.threshold.torque.x = 10
            req.threshold.torque.y = 10
            req.threshold.torque.z = 10
            
            resp = detect_fts(req)
            print("FTS Detector Response:", resp.result)
            result = json.loads(resp.result)
            return (result["starting_forces"][2]>req.threshold.force.z)
        except rospy.ServiceException as e:
            print("Service call failed:", e)
            return None

    
    # def send_end_vision_outcome_request(self, param):
    #     return -1
    #     raise NotImplementedError
    
    def send_end_vision_outcome_request(self, param):
        rospy.wait_for_service('/lego_detector')
        try:
           
            detect_lego = rospy.ServiceProxy('/lego_detector', LegoOutcome)
            
        
            req = LegoOutcomeRequest()
            req.id = 0  
            req.topic_name = '/side_camera/color/image_cropped'
            req.start = False 
            req.score_threshold = 0.8  
            
            top_bbox = BoundingBox()
            top_bbox.coords = [800, 200, 1100, 350]  
            bot_bbox = BoundingBox()
            bot_bbox.coords = [800, 350, 1100, 500] 
            req.top_bbox = top_bbox
            req.bot_bbox = bot_bbox
            
            resp = detect_lego(req)
            lego_result = json.loads(resp.result)
            print("Vision Detection Response:", resp.result)
           
            return lego_result["ending_top"] == 0 and lego_result["ending_bottom"] == 1
        
        except rospy.ServiceException as e:
            print("Vision service call failed:", e)
            return None

class PlaceLegoSkill(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, namespace: str):

        super().__init__(robot_commander, namespace)

        self.skill_steps = [
            {'step_name': 'go_to_approach_pose',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.approach_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.approach_pose_msg),
             'outcome': lambda param: self.send_start_outcome_request(param)},
            {'step_name': 'engage_brick',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.place_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(engage_termination_config, self.place_pose_msg)},
            {'step_name': 'pull_up',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.servo_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(servo_termination_config, self.servo_pose_msg),
             'outcome': lambda param: self.send_end_fts_outcome_request(param)},
            {'step_name': 'reengage_brick',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.place_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(engage_termination_config, self.place_pose_msg)},
            {'step_name': 'release_brick',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.release_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(release_termination_config, self.release_pose_msg),
             'outcome': lambda param: self.send_end_vision_outcome_request(param)},
            {'step_name': 'disengage_brick',
             'robot_command': lambda param: self.robot_commander.go_to_pose_goal(self.disengage_pose_msg, wait=False),
             'termination_cfg': lambda param: add_termination_pose(rapid_termination_config, self.disengage_pose_msg)}
        ]

    def execute_skill(self, execution_params, skill_params) -> Tuple[List[TerminationSignal], List[int]]:
        # QOL 
        if 'T_lego_ee' not in skill_params:
            print(f"PlaceLegoSkill expects end effector transform: skill_params['T_lego_ee'] = RigidTransform()")
        if 'T_lego_world' not in skill_params:
            print(f"PlaceLegoSkill expects target brick pose transform: skill_params['T_lego_world'] = RigidTransform()")
        if 'approach_height_offset' not in skill_params:
            print(f"PlaceLegoSkill expects an approach height offset (meters): skill_params['approach_height_offset'] = float(0.010 m)")
        if 'place_rotation' not in skill_params:
            print(f"PlaceLegoSkill expects a release rotation (deg): skill_params['place_rotation'] = float(30 deg)")

        self.T_lego_ee = skill_params['T_lego_ee']
        self.T_lego_world = skill_params['T_lego_world']
        self.approach_height_offset = skill_params['approach_height_offset']
        self.place_rotation = skill_params['place_rotation']

        self.T_lego_approach = RigidTransform(
            translation=np.array([0.0, 0.0, -abs(self.approach_height_offset)]),
            from_frame='lego', to_frame='lego'
        )
        self.T_lego_rotation = RigidTransform(
            rotation=RigidTransform.y_axis_rotation(np.deg2rad(self.place_rotation)), 
            translation=np.array([LEGO_BLOCK_WIDTH/2,0,0]), 
            from_frame='lego', to_frame='lego'
        )
        self.T_lego_rotation_point_offset = RigidTransform(
            translation=np.array([LEGO_BLOCK_WIDTH/2,0,0]), 
            from_frame='lego', to_frame='lego'
        )
        self.T_lego_servo = RigidTransform(
            translation=np.array([0.0, 0.0, -0.001]),
            from_frame='lego', to_frame='lego'
        )
        self.T_lego_disengage = RigidTransform(

        )


        self.T_place_target = self.T_lego_world.copy()
        if skill_params['place_perturbation']:
            self.place_perturbation = np.array(skill_params['place_perturbation'])
            self.T_place_target *= RigidTransform(
                translation=self.place_perturbation, 
                from_frame='lego', to_frame='lego'
            )
        
        self.place_pose = self.T_place_target * self.T_lego_ee.inverse()
        self.place_pose_msg = self.place_pose.pose_msg

        self.approach_pose = self.T_place_target * self.T_lego_approach * self.T_lego_ee.inverse()
        self.approach_pose_msg = self.approach_pose.pose_msg

        self.release_pose = self.T_place_target * self.T_lego_rotation * self.T_lego_rotation_point_offset.inverse() * self.T_lego_ee.inverse()
        self.release_pose_msg = self.release_pose.pose_msg

        self.servo_pose = self.T_place_target * self.T_lego_servo * self.T_lego_ee.inverse()
        self.servo_pose_msg = self.servo_pose.pose_msg

        self.disengage_pose = self.T_place_target * self.T_lego_approach
        self.disengage_pose *= self.T_lego_rotation * self.T_lego_rotation_point_offset.inverse() * self.T_lego_ee.inverse()
        self.disengage_pose_msg = self.disengage_pose.pose_msg

        return super().execute_skill(execution_params)

    def send_start_outcome_request(self, param):
        rospy.wait_for_service('/fts_detector')
        rospy.wait_for_service('/lego_detector')

        try:
            
            detect_fts = rospy.ServiceProxy('/fts_detector', FTSOutcome)
            fts_req = FTSOutcomeRequest()
            fts_req.id = 0
            fts_req.topic_name = 'fts'  
            fts_req.start = True
            fts_req.threshold = Wrench()
            fts_req.threshold.force.x = 10
            fts_req.threshold.force.y = 10
            fts_req.threshold.force.z = 2.0
            fts_req.threshold.torque.x = 10
            fts_req.threshold.torque.y = 10
            fts_req.threshold.torque.z = 10
            

            fts_resp = detect_fts(fts_req)
            fts_result = json.loads(fts_resp.result)
            print("FTS Detector Response:", fts_resp.result)
            

            detect_lego = rospy.ServiceProxy('/lego_detector', LegoOutcome)
            lego_req = LegoOutcomeRequest()
            lego_req.id = 0
            lego_req.topic_name = '/side_camera/color/image_cropped'
            lego_req.start = True
            lego_req.score_threshold = 0.8
            
            top_bbox = BoundingBox(coords=[800, 200, 1100, 350])
            bot_bbox = BoundingBox(coords=[800, 350, 1100, 500])
            lego_req.top_bbox = top_bbox
            lego_req.bot_bbox = bot_bbox

            lego_resp = detect_lego(lego_req)
            lego_result = json.loads(lego_resp.result)
            print("Vision Detection Response:", lego_resp.result)
            return lego_result["starting_top"] ==0 and lego_result["starting_bottom"] ==0 and fts_result["starting_forces"][2]<fts_req.threshold.force.z
    

        except rospy.ServiceException as e:
            print("Service call failed:", e)
            return None


    
    # def send_end_fts_outcome_request(self, param):
    #     return -1
    #     raise NotImplementedError
    
    def send_end_fts_outcome_request(self, param):
        rospy.wait_for_service('/fts_detector')
        try:
          
            detect_fts = rospy.ServiceProxy('/fts_detector', FTSOutcome)
            
            
            req = FTSOutcomeRequest()
            req.id = 0  
            req.topic_name ='fts'
            req.start = False  
            req.threshold = Wrench()
            
       
            req.threshold.force.x = 10
            req.threshold.force.y = 10
            req.threshold.force.z =2.0
            req.threshold.torque.x = 10
            req.threshold.torque.y = 10
            req.threshold.torque.z = 10
            
            resp = detect_fts(req)
            print("FTS Detector Response:", resp.result)
            result = json.loads(resp.result)
            return (result["starting_forces"][2]>req.threshold.force.z)
        except rospy.ServiceException as e:
            print("Service call failed:", e)
            return None

    
    # def send_end_vision_outcome_request(self, param):
    #     return -1
    #     raise NotImplementedError
    
    def send_end_vision_outcome_request(self, param):
        rospy.wait_for_service('/lego_detector')
        try:
           
            detect_lego = rospy.ServiceProxy('/lego_detector', LegoOutcome)
            
        
            req = LegoOutcomeRequest()
            req.id = 0  
            req.topic_name = '/side_camera/color/image_cropped'
            req.start = False 
            req.score_threshold = 0.8  
            
            top_bbox = BoundingBox()
            top_bbox.coords = [800, 200, 1100, 350]  
            bot_bbox = BoundingBox()
            bot_bbox.coords = [800, 350, 1100, 500] 
            req.top_bbox = top_bbox
            req.bot_bbox = bot_bbox
            
            resp = detect_lego(req)
            lego_result = json.loads(resp.result)
            print("Vision Detection Response:", resp.result)
           
            return lego_result["ending_top"] == 0 and lego_result["ending_bottom"] == 1
        
        except rospy.ServiceException as e:
            print("Vision service call failed:", e)
            return None



    def calculate_approach_pose(self, param):
        place_perturbation = np.array(param['place_perturbation_mm']) / 1000.0
        T_place_offset = RigidTransform(
            translation=place_perturbation, 
            from_frame='lego', to_frame='lego'
        )
        T_place_target = self.T_lego_world * T_place_offset 
        approach_pose = T_place_target * self.T_lego_approach * self.T_lego_ee.inverse()
        approach_pose = approach_pose.pose_msg
        self.robot_commander.go_to_pose_goal(approach_pose, wait=False)

class PickLegoSkill(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander, params):

        super().__init__(robot_commander)

        self.T_lego_ee = params['T_lego_ee']
        self.T_lego_world = params['T_lego_world']

        T_lego_approach = RigidTransform(
            translation=np.array([0.0, 0.0, -0.050])
        )
        T_lego_rotation = RigidTransform(
            rotation=RigidTransform.y_axis_rotation(np.deg2rad(-15)), 
            translation=np.array([LEGO_BLOCK_WIDTH/2,0,LEGO_BLOCK_HEIGHT]), 
            from_frame='lego', to_frame='lego'
        )
        T_lego_rotation_point_offset = RigidTransform(
            translation=np.array([LEGO_BLOCK_WIDTH/2,0,LEGO_BLOCK_HEIGHT]), 
            from_frame='lego', to_frame='lego'
        )
        T_lego_servo = RigidTransform(
            translation=np.array([0.0, 0.0, -0.001]),
            from_frame='lego', to_frame='lego'
        )


        T_place_target = T_lego_world
        if params['place_perturbation_mm']:
            place_perturbation = np.array(params['place_perturbation_mm']) / 1000.0
            # Should this be a perturbation in the world frame or EE frame?
            T_place_target *= RigidTransform(
                translation=place_perturbation, 
                from_frame='lego', to_frame='lego'
            )
        
        place_pose = T_place_target * T_lego_ee.inverse()
        place_pose = place_pose.pose_msg

        approach_pose = T_place_target * T_lego_approach * T_lego_ee.inverse()
        approach_pose = approach_pose.pose_msg

        release_pose = T_place_target * T_lego_rotation 
        release_pose *= T_lego_rotation_point_offset.inverse() * T_lego_ee.inverse()
        release_pose = release_pose.pose_msg

        servo_pose = T_place_target * T_lego_servo * T_lego_ee.inverse()
        servo_pose = servo_pose.pose_msg


        self.skill_steps = [
            {'step_name': 'go_to_approach_pose',
             'command': lambda: self.robot_commander.go_to_pose_goal(approach_pose, wait=False),
             'termination_cfg': add_termination_pose(rapid_termination_config, approach_pose)},
            {'step_name': 'engage_brick',
             'command': lambda: self.robot_commander.go_to_pose_goal(place_pose, wait=False),
             'termination_cfg': add_termination_pose(engage_termination_config, place_pose)},
            {'step_name': 'pull_up',
             'command': lambda: self.robot_commander.go_to_pose_goal(servo_pose, wait=False),
             'termination_cfg': add_termination_pose(servo_termination_config, servo_pose)},
            {'step_name': 'reengage_brick',
             'command': lambda: self.robot_commander.go_to_pose_goal(place_pose, wait=False),
             'termination_cfg': add_termination_pose(engage_termination_config, place_pose)},
            {'step_name': 'release_brick',
             'command': lambda: self.robot_commander.go_to_pose_goal(release_pose, wait=False),
             'termination_cfg': add_termination_pose(release_termination_config, release_pose)},
            {'step_name': 'disengage_brick',
             'command': lambda: self.robot_commander.go_to_pose_goal(approach_pose, wait=False),
             'termination_cfg': add_termination_pose(rapid_termination_config, approach_pose)}
        ]

class ResetLegoPalletSkill(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander):

        super().__init__(robot_commander)

        self.skill_steps = [
             {'step_name': 'go_to_home_joints',
              'command': lambda: self.robot_commander.go_to_joints([0, 0, 0, 0, -np.pi/2, 0], wait=False),
              'termination_cfg': {'joint': {'positions': [0, 0, 0, 0, -np.pi/2, 0]}}}
        ]

class ResetLegoGripperSkill(BaseSkill):

    def __init__(self, robot_commander: BaseRobotCommander):

        super().__init__(robot_commander)

        self.skill_steps = [
             {'step_name': 'go_to_home_joints',
              'command': lambda: self.robot_commander.go_to_joints([0, 0, 0, 0, -np.pi/2, 0], wait=False),
              'termination_cfg': {'joint': {'positions': [0, 0, 0, 0, -np.pi/2, 0]}}}
        ]