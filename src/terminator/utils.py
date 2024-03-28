#!/usr/bin/env python3

import json

from terminator.base_termination_handler import BaseTerminationHandler
from terminator.FTS_termination_handler import FTSTerminationHandler
from terminator.timeout_termination_handler import TimeoutTerminationHandler

from geometry_msgs.msg import Wrench
from vibro_tactile_toolbox.msg import TerminationConfig

def get_handler_from_name(handler_name: str):
    if handler_name == 'AbstractBase':
        return BaseTerminationHandler()
    elif handler_name == 'FTS':
        return FTSTerminationHandler()
    elif handler_name == 'timeout':
        return TimeoutTerminationHandler()


### TerminationConfig Helpers ###
'''
TerminationConfig Example
{
    'id': int 0,
    'timeout': {
        'timeout_duration_ns': int 10E9
    },
    'FTS': {
        'check_rate_ns': int 1E6
        'threshold': Wrench(force={30, 30, 30}, torque={1.5, 1.5, 1.5})
    },
    'robot': {
        TBD
    },
    'audio': {
        TBD
    },
    vision: {
        TBD
    }
}

'''    


def make_termination_config(id, timeout_cfg=None, FTS_cfg=None, robot_cfg=None, audio_cfg=None, vision_cfg=None) -> TerminationConfig:
    termination_config = TerminationConfig()
    cfg_json = {'id': id}
    if timeout_cfg:
        cfg_json['timeout'] = timeout_cfg
    if FTS_cfg:
        cfg_json['FTS'] = FTS_cfg
    if robot_cfg:
        cfg_json['robot'] = robot_cfg
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