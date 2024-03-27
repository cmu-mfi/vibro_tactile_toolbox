#!/usr/bin/env python3

from terminator.base_termination_handler import BaseTerminationHandler
from terminator.FTS_termination_handler import FTSTerminationHandler

def get_handler_from_name(handler_name: str):
    if handler_name == 'AbstractBase':
        return BaseTerminationHandler()
    elif handler_name == 'FTS':
        return FTSTerminationHandler()