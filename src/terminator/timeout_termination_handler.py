#!/usr/bin/env python3

import rospy
import json

from vibro_tactile_toolbox.msg import TerminationConfig, TerminationSignal
from terminator.base_termination_handler import BaseTerminationHandler

class TimeoutTerminationHandler(BaseTerminationHandler):
    """
    Termination handler to stop on a timeout condition
    """
    def __init__(self):
        self.id = -1
        self.input_data_class = None
        self.check_rate_ns = 100E6 # 100ms default

        self.timeout_duration_ns = 10E9

        self.start_time = None

    def update_config(self, cfg: TerminationConfig):
        """
        Callback to update the termination handler config from a json
        """
        cfg_jsons = cfg.cfg_json
        cfg_json = json.loads(cfg_jsons)
        if 'timeout' in cfg_json:
            self.id = cfg_json['id']
            timeout_cfg = cfg_json['timeout']
            if 'timeout_duration_ns' in timeout_cfg:
                self.timeout_duration_ns = timeout_cfg['timeout_duration_ns']
            # Always reset the time if a timeout termination is requested
            self.start_time = rospy.get_time()

    
    def update_input_data(self, input_signal):
        """
        Callback to update the input signal
        """
        # No input signal, we'll keep track of time internally since there's no topic to subscribe to
        pass
    
    def get_termination_signal(self) -> TerminationSignal:
        """
        Callback to produce the termination signal
        """
        termination_signal = TerminationSignal()
        curr_time = rospy.get_time()
        terminate = (self.start_time is not None) and (curr_time - self.start_time) > self.timeout_duration_ns / 1E9
        cause = f"Timeout termination handler caused by:\n"
        if terminate:
            cause += f"Command execution timeout after {(curr_time - self.start_time):0.2f}s with timeout duration {self.timeout_duration_ns/1E9:0.2f}s"
        
        termination_signal.terminate = terminate
        termination_signal.cause = cause
        return termination_signal