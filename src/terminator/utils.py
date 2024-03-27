#!/usr/bin/env python3

from terminator.base_termination_handler import BaseTerminationHandler
from terminator.FTS_termination_handler import FTSTerminationHandler

from geometry_msgs.msg import Wrench

def get_handler_from_name(handler_name: str):
    if handler_name == 'AbstractBase':
        return BaseTerminationHandler()
    elif handler_name == 'FTS':
        return FTSTerminationHandler()


### TerminationConfig Helpers ###

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