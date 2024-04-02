#!/usr/bin/env python3

import json

from terminator.base_termination_handler import BaseTerminationHandler
from terminator.fts_termination_handler import FTSTerminationHandler
from terminator.time_termination_handler import TimeTerminationHandler
from terminator.pose_termination_handler import PoseTerminationHandler
from terminator.joint_termination_handler import JointTerminationHandler

from geometry_msgs.msg import Wrench, Pose
from sensor_msgs.msg import JointState
from vibro_tactile_toolbox.msg import TerminationConfig

def get_handler_from_name(handler_name: str):
    if handler_name == 'AbstractBase':
        return BaseTerminationHandler()
    elif handler_name == 'fts':
        return FTSTerminationHandler()
    elif handler_name == 'time':
        return TimeTerminationHandler()
    elif handler_name == 'joint':
        return JointTerminationHandler()
    elif handler_name == 'pose':
        return PoseTerminationHandler()


### TerminationConfig Helpers ###
'''
TerminationConfig Example
{
    'id': int 0,
    'time': {
        'duration': float 10.0
    },
    'fts': {
        'check_rate_ns': int 1E6
        'threshold': Wrench(force={30, 30, 30}, torque={1.5, 1.5, 1.5})
    },
    'joint': {
        'tolerance': 0.001
        'position': [0, 0, 0, 0, -np.pi/2, 0]
    },
    'pose': {
        'pos_tolerance': 0.001
        'orient_tolerance': 0.01
        'pose': Pose(position={0.29, 0, 0.533}, orientation={-1, 0, 0, 0})
    },
    'audio': {
        TBD
    },
    vision: {
        TBD
    }
}

'''    


def make_termination_config(id, time_cfg=None, fts_cfg=None, joint_cfg=None, pose_cfg=None, audio_cfg=None, vision_cfg=None) -> TerminationConfig:
    termination_config = TerminationConfig()
    cfg_json = {'id': id}
    if time_cfg:
        cfg_json['time'] = time_cfg
    if FTS_cfg:
        cfg_json['fts'] = fts_cfg
    if joint_cfg:
        cfg_json['joint'] = joint_cfg
    if pose_cfg:
        cfg_json['pose'] = pose_cfg
    if audio_cfg:
        cfg_json['audio'] = audio_cfg
    if vision_cfg:
        cfg_json['vision'] = vision_cfg
    cfg_jsons = json.dumps(cfg_json)
    termination_config.cfg_json = cfg_jsons
    return termination_config

def wrench_to_dict(msg: Wrench):
    # Convert ROS message to dictionary
    msg_dict = {
        'force': {
            'x': msg.force.x,
            'y': msg.force.y,
            'z': msg.force.z
        },
        'torque': {
            'x': msg.torque.x,
            'y': msg.torque.y,
            'z': msg.torque.z
        }
    }
    return msg_dict

def dict_to_wrench(msg_dict: dict) -> Wrench:
    msg = Wrench()
    msg.force.x = msg_dict['force']['x']
    msg.force.y = msg_dict['force']['y']
    msg.force.z = msg_dict['force']['z']
    msg.torque.x = msg_dict['torque']['x']
    msg.torque.y = msg_dict['torque']['y']
    msg.torque.z = msg_dict['torque']['z']
    return msg

def joint_state_to_dict(msg: JointState):
    # Convert ROS message to dictionary
    msg_dict = {
        'name': msg.name,
        'position': msg.position,
        'velocity': msg.velocity,
        'effort': msg.effort
    }
    return msg_dict

def dict_to_joint_state(msg_dict: dict) -> JointState:
    msg = JointState()
    if 'name' in msg_dict:
        msg.name = msg_dict['name']
    if 'position' in msg_dict:
        msg.position = msg_dict['position']
    if 'velocity' in msg_dict:
        msg.velocity = msg_dict['velocity']
    if 'effort' in msg_dict:
        msg.effort = msg_dict['effort']
    return msg

def pose_to_dict(msg: Pose):
    # Convert ROS message to dictionary
    msg_dict = {
        'position': {
            'x': msg.position.x,
            'y': msg.position.y,
            'z': msg.position.z
        },
        'orientation': {
            'x': msg.orientation.x,
            'y': msg.orientation.y,
            'z': msg.orientation.z,
            'w': msg.orientation.w
        }
    }
    return msg_dict

def dict_to_pose(msg_dict: dict) -> Pose:
    msg = Pose()
    msg.position.x = msg_dict['position']['x']
    msg.position.y = msg_dict['position']['y']
    msg.position.z = msg_dict['position']['z']
    msg.orientation.x = msg_dict['orientation']['x']
    msg.orientation.y = msg_dict['orientation']['y']
    msg.orientation.z = msg_dict['orientation']['z']
    msg.orientation.w = msg_dict['orientation']['w']
    return msg