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

from typing import Tuple, List

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
        'duration':         (float) - timeout duration in seconds
    },
    'fts':  {
        'check_rate_ns':    (int) - rate in ns to check the fts for termination
        'threshold': {      (dict) - bilateral range for non-terminal values
            'force: {       (dict) - force ramges
                'x':        (List[float]) - x range
                'y':        (List[float]) - y range 
                'z':        (List[float]) - z range
            }
            'torque: {      (dict) - torque ranges
                'x':        (List[float]) - x range
                'y':        (List[float]) - y range 
                'z':        (List[float]) - z range        
            }
        }
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
    if fts_cfg:
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

def dict_to_wrench_bilateral(msg_dict: dict) -> Tuple[Wrench]:
    msg_lo = Wrench()
    msg_hi = Wrench()

    msg_lo.force.x = min(msg_dict['force']['x'])
    msg_hi.force.x = max(msg_dict['force']['x'])

    msg_lo.force.y = min(msg_dict['force']['y'])
    msg_hi.force.y = max(msg_dict['force']['y'])

    msg_lo.force.z = min(msg_dict['force']['z'])
    msg_hi.force.z = max(msg_dict['force']['z'])

    msg_lo.torque.x = min(msg_dict['torque']['x'])
    msg_hi.torque.x = max(msg_dict['torque']['x'])

    msg_lo.torque.y = min(msg_dict['torque']['y'])
    msg_hi.torque.y = max(msg_dict['torque']['y'])

    msg_lo.torque.z = min(msg_dict['torque']['z'])
    msg_hi.torque.z = max(msg_dict['torque']['z'])
    return msg_lo, msg_hi

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